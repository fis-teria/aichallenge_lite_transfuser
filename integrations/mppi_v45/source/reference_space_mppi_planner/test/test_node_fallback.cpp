// The generated adapter changes access and the entrypoint only, not behavior.
#pragma GCC diagnostic ignored "-Wsubobject-linkage"
#include "node_test_access.hpp"
#include <gtest/gtest.h>
#include <future>

namespace reference_space_mppi_planner {
class NodeFallbackTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {int argc=0; rclcpp::init(argc,nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  ReferenceSpaceMppiNode node;
  void stopBrainWorker() {
    {
      std::lock_guard<std::mutex> lock(node.work_mutex_);
      node.stopping_ = true;
    }
    node.work_cv_.notify_one();
    if (node.worker_.joinable()) node.worker_.join();
  }
  Command prepareBrainScene(double lateral = 0.0) {
    stopBrainWorker();
    node.set_parameter(rclcpp::Parameter("use_sim_time", true));
    node.brain_mode_ = node.enabled_ = node.config_.enabled = true;
    node.brain_cruise_speed_mps_ = 4.0;
    auto command = desired();
    command.header.stamp = node.now();
    command.valid_until_sec = node.now().seconds() + 0.2;
    node.latest_generation_.store(command.generation);
    node.base_reference_ = std::make_shared<Trajectory>(command.trajectory);
    auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    map->info.resolution = 0.25; map->info.width = map->info.height = 400;
    map->info.origin.position.x = map->info.origin.position.y = -40.0;
    map->data.assign(160000, 0);
    node.wall_map_ = map;
    node.wall_index_ = std::make_shared<OccupancyGridWallIndex>(map);
    nav_msgs::msg::Odometry ego;
    ego.header.stamp = node.now(); ego.pose.pose.orientation.w = 1.0;
    ego.pose.pose.position.y = lateral; ego.twist.twist.linear.x = 4.0;
    node.odometry_ = ego;
    return command;
  }
  Command desired() {
    Command c; c.generation=42; c.valid_until_sec=123.; c.mode="OVERTAKE_RIGHT";
    c.header.stamp.sec=122; c.reason="test";
    for (int i=0; i<80; ++i) {
      autoware_auto_planning_msgs::msg::TrajectoryPoint p;
      p.pose.position.x=i*.5; p.pose.orientation.w=1.; p.longitudinal_velocity_mps=10.;
      c.trajectory.points.push_back(p);
    }
    return c;
  }
  Command followAt(double body_gap, const std::string &id = "lead", double speed = 1.) {
    const auto base = desired();
    nav_msgs::msg::Odometry ego;
    ego.pose.pose.position.x = 5.;
    ego.pose.pose.orientation.w = 1.;
    ego.twist.twist.linear.x = speed;
    BrainInputs inputs;
    ObservedVehicle lead;
    lead.id = id;
    lead.x_m = 5. + node.brain_footprint_front_m_ +
                   node.brain_footprint_rear_m_ + body_gap;
    lead.vx_mps = speed;
    inputs.opponents.push_back(lead);
    const auto target = node.selectFollowVehicle(base.trajectory, base.trajectory, ego, inputs);
    EXPECT_TRUE(target.has_value());
    return target ? node.makeBrainFollowCommand(base, target, node.driving_fsm_) : Command{};
  }
};

#include "reference_contract_tests.inc"

class NodePreparationBoostTest : public NodeFallbackTest {
protected:
  BrainInputs prepareBoostScene(int lap = 1) {
    prepareBrainScene();
    node.preparation_boost_config_.enabled = true;
    node.preparation_boost_pub_ =
        node.create_publisher<std_msgs::msg::Float32MultiArray>("/test/preparation_boost", 10);
    node.receivePreparationBoostState("Ready");
    node.receivePreparationBoostState("Start");
    std_msgs::msg::Float32MultiArray status;
    status.data = {300.F, static_cast<float>(lap), 0.F, 0.F, 1.F, 2.F, 0.F};
    node.receivePreparationBoostStatus(status);
    auto plan = std::make_shared<mppi::PassingPreparationPlan>();
    plan->id = plan->revision = 41;
    plan->target_id = "lead";
    plan->schedule.push_back({0., 0., 4., 1.});
    BrainInputs inputs;
    inputs.input_sequence = 12;
    inputs.recovery_epoch = node.recovery_epoch_;
    inputs.preparation = node.latest_preparation_ = plan;
    node.latest_preparation_input_ = inputs.input_sequence;
    return inputs;
  }
};

TEST_F(NodeFallbackTest, SnapshotSharesPublishedReferenceAndCausalDelayAcrossCandidates) {
  auto command=prepareBrainScene();
  node.preparation_planning_delay_sec_.store(.12);
  node.publishOutput(command);
  const auto state_stamp=rclcpp::Time(node.odometry_->header.stamp).seconds();
  const auto lower_age=std::max(0.,node.now().seconds()-state_stamp);
  const auto inputs=node.captureBrainInputs();
  const auto upper_age=std::max(0.,node.now().seconds()-state_stamp);
  ASSERT_TRUE(inputs.prior_execution_reference);
  EXPECT_GE(inputs.reference_activation_delay_sec,.12+lower_age);
  EXPECT_LE(inputs.reference_activation_delay_sec,.12+upper_age);
  const auto shared=node.executionContext(inputs,*inputs.odometry);
  ASSERT_TRUE(shared);
  mppi::PlanRequest left,right;left.side=1;right.side=-1;
  const auto a=node.executionRequestForCandidate(*shared,left);
  const auto b=node.executionRequestForCandidate(*shared,right);
  EXPECT_EQ(a.prior_execution_reference,b.prior_execution_reference);
  EXPECT_DOUBLE_EQ(a.reference_activation_delay_sec,b.reference_activation_delay_sec);
  command.trajectory.points.front().longitudinal_velocity_mps=2.;
  node.publishOutput(command);
  EXPECT_FLOAT_EQ(inputs.prior_execution_reference->points[0].speed_mps,10.);
  EXPECT_FLOAT_EQ(node.captureBrainInputs().prior_execution_reference->points[0].speed_mps,2.);
}

TEST_F(NodePreparationBoostTest, PreparedGoalPublishesSingleEdgePairBeforePathAdoption) {
  auto inputs = prepareBoostScene();
  ASSERT_FALSE(inputs.preparation->connection);
  ASSERT_FALSE(node.active_brain_command_);
  auto observer = std::make_shared<rclcpp::Node>("preparation_boost_observer");
  std::vector<float> edges;
  auto subscription = observer->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/test/preparation_boost", rclcpp::QoS(10),
      [&](std_msgs::msg::Float32MultiArray::ConstSharedPtr message) {
        if (!message->data.empty()) edges.push_back(message->data[0]);
      });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (node.preparation_boost_pub_->get_subscription_count() == 0U &&
         std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_GT(node.preparation_boost_pub_->get_subscription_count(), 0U);
  EXPECT_TRUE(node.maybeFirePreparationBoost(inputs));
  EXPECT_FALSE(node.maybeFirePreparationBoost(inputs));
  while (edges.size() < 2U && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(observer);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_EQ(edges, (std::vector<float>{1.F, 0.F}));
  EXPECT_TRUE(node.preparation_boost_policy_.fired[0]);
  EXPECT_FALSE(node.preparation_boost_policy_.fired[1]);
}

TEST_F(NodePreparationBoostTest, UnreadyOrSupersededGoalAndRecoveryCannotFire) {
  auto inputs = prepareBoostScene();
  node.odometry_->pose.pose.position.x = 35.;  // wp70, outside the requested interval
  EXPECT_FALSE(node.maybeFirePreparationBoost(inputs));
  node.odometry_->pose.pose.position.x = 0.;
  auto absent = inputs; absent.preparation.reset();
  EXPECT_FALSE(node.maybeFirePreparationBoost(absent));
  ++node.latest_preparation_input_;
  EXPECT_FALSE(node.maybeFirePreparationBoost(inputs));
  node.latest_preparation_input_ = inputs.input_sequence;
  node.recovery_active_ = true;
  EXPECT_FALSE(node.maybeFirePreparationBoost(inputs));
  node.recovery_active_ = false;
  ++node.recovery_epoch_;
  EXPECT_FALSE(node.maybeFirePreparationBoost(inputs));
  inputs.recovery_epoch = node.recovery_epoch_;
  node.shadow_only_ = true;
  EXPECT_FALSE(node.maybeFirePreparationBoost(inputs));
  node.shadow_only_ = false;
  EXPECT_TRUE(node.maybeFirePreparationBoost(inputs));
}

TEST_F(NodePreparationBoostTest, OfficialStatusSelectsLateAllocationAndRequiresBoostIdle) {
  auto inputs = prepareBoostScene(5);
  std_msgs::msg::Float32MultiArray status;
  status.data = {300.F, 5.F, 0.F, 0.F, 1.F, 1.F, 1.F};
  node.receivePreparationBoostStatus(status);
  EXPECT_FALSE(node.maybeFirePreparationBoost(inputs));
  status.data[6] = 0.F;
  node.receivePreparationBoostStatus(status);
  EXPECT_TRUE(node.maybeFirePreparationBoost(inputs));
  EXPECT_FALSE(node.preparation_boost_policy_.fired[0]);
  EXPECT_TRUE(node.preparation_boost_policy_.fired[1]);
  status.data[1] = 6.F;
  node.receivePreparationBoostStatus(status);
  EXPECT_FALSE(node.maybeFirePreparationBoost(inputs));
}

TEST_F(NodeFallbackTest, AbortReturnIsConnectedValidatedAndOwnedUntilAlignment) {
  auto command=prepareBrainScene(2.);
  node.driving_fsm_.acceptLateral(DrivingMode::OVERTAKE);
  node.odometry_->pose.pose.orientation=quaternionFromYaw(.15);
  node.odometry_->twist.twist.linear.x=2.;
  auto active=command;active.mode="OVERTAKE";active.corridor_side=1;
  for(auto &p:active.trajectory.points) {p.pose.position.y=2.;p.longitudinal_velocity_mps=2.;}
  node.active_brain_command_=active;
  for(auto &p:command.trajectory.points)p.longitudinal_velocity_mps=2.;
  int validated=0;
  const auto validator=[&](const mppi::TemporaryReference &r) {
    ++validated;
    return std::abs(r.points.front().y_m-2.)<1e-9 ? mppi::RejectReason::NONE : mppi::RejectReason::WALL;
  };
  const auto returned=node.connectedReturnFallback(command,*node.odometry_,.05,
      *node.base_reference_,active,DrivingMode::OVERTAKE,validator);
  ASSERT_TRUE(node.isConnectedReturn(returned));EXPECT_GT(validated,0);
  EXPECT_EQ(returned.corridor_side,0);
  const auto &a=returned.trajectory.points[0].pose.position,&b=returned.trajectory.points[1].pose.position;
  EXPECT_NEAR(std::atan2(b.y-a.y,b.x-a.x),.15,.05);
  node.adoptBrainReturn(returned);
  ASSERT_TRUE(node.active_brain_command_);EXPECT_EQ(node.active_brain_command_->trajectory,returned.trajectory);
  const auto shape=node.active_brain_command_->generation;
  node.receiveBrainCommand(command);
  ASSERT_TRUE(node.pending_batch_);ASSERT_EQ(node.pending_batch_->candidate_count,1U);
  EXPECT_EQ(node.pending_batch_->candidates.front().request.phase,mppi::Phase::MERGE);
  EXPECT_TRUE(node.pending_batch_->allow_continuation);
  ASSERT_TRUE(node.active_brain_command_);EXPECT_EQ(node.active_brain_command_->generation,shape);
}

TEST_F(NodeFallbackTest, AlongsideDetectionIncludesBothSidesAndJustPassedCenters) {
  prepareBrainScene(2.);
  node.odometry_->pose.pose.position.x=10.;
  ObservedVehicle other;other.id="adjacent";other.vx_mps=4.;
  for(double y:{0.,4.}) for(double x:{9.8,10.2}) {
    other.x_m=x;other.y_m=y;
    EXPECT_TRUE(node.hasAlongsideVehicle(*node.base_reference_,*node.odometry_,{other}));
  }
  for(double x:{6.,14.}) {
    other.x_m=x;other.y_m=0.;
    EXPECT_FALSE(node.hasAlongsideVehicle(*node.base_reference_,*node.odometry_,{other}));
  }
  other.x_m=10.;other.y_m=2.;
  EXPECT_FALSE(node.hasAlongsideVehicle(*node.base_reference_,*node.odometry_,{other}));
}

TEST_F(NodeFallbackTest, AlongsideDetectionWrapsAtClosedReferenceSeam) {
  prepareBrainScene();
  Trajectory circle;
  for(int i=0;i<=200;++i) {
    const double angle=2.*M_PI*i/200.;
    autoware_auto_planning_msgs::msg::TrajectoryPoint p;
    p.pose.position.x=50.*std::cos(angle);p.pose.position.y=50.*std::sin(angle);
    circle.points.push_back(p);
  }
  node.odometry_->pose.pose.position.x=52.;node.odometry_->pose.pose.position.y=.1;
  node.odometry_->pose.pose.orientation=quaternionFromYaw(M_PI/2.);
  ObservedVehicle other;other.x_m=50.*std::cos(-.01);other.y_m=50.*std::sin(-.01);
  other.vx_mps=0.;other.vy_mps=4.;
  EXPECT_TRUE(node.hasAlongsideVehicle(circle,*node.odometry_,{other}));
}

TEST_F(NodeFallbackTest, FailedBatchAlongsideKeepsCommittedPathAcrossTargetSwitch) {
  const auto desired_command=prepareBrainScene(2.);
  node.driving_fsm_.acceptLateral(DrivingMode::AVOID);
  Command active=desired_command;active.generation=17;active.mode="AVOID";active.corridor_side=1;
  for(auto &p:active.trajectory.points)p.pose.position.y=2.;
  node.active_brain_command_=active;
  WorkBatch batch;batch.brain_owned=true;batch.maneuver=DrivingMode::OVERTAKE;
  batch.maneuver_target_id="new_leader";batch.allow_continuation=false;
  batch.fallback=desired_command;batch.brain_hold=active;batch.global_reference=node.base_reference_;
  ObservedVehicle other;other.id="old_leader";other.x_m=-.177;other.y_m=0.;other.vx_mps=.8;
  batch.opponents={other};
  WorkItem work;work.odometry=*node.odometry_;batch.candidates.push_back(work);
  batch.fallback_path_constraint_validator=[](const auto &){return mppi::RejectReason::NONE;};
  const auto fallback=node.makeBatchFallback(batch,std::nullopt);
  ASSERT_GE(fallback.trajectory.points.size(),3U);
  EXPECT_EQ(fallback.reason,"mppi_brain:alongside_hold");
  EXPECT_FALSE(node.isConnectedReturn(fallback));EXPECT_EQ(fallback.corridor_side,1);
  for(const auto &p:fallback.trajectory.points)EXPECT_DOUBLE_EQ(p.pose.position.y,2.);
  EXPECT_EQ(node.active_brain_command_->generation,17U);
  EXPECT_EQ(node.active_brain_command_->trajectory,active.trajectory);
}

TEST_F(NodeFallbackTest, AlongsideFallbackUsesInstalledSpeedProfileWithoutChangingGeometry) {
  const auto command=prepareBrainScene(2.);
  auto active=command;active.corridor_side=1;
  for(auto &p:active.trajectory.points)p.pose.position.y=2.;
  auto profile=mppi::executionSpeedProfile(node.temporaryReferenceFromCommand(active));
  for(std::size_t i=0;i<profile.count;++i)profile.points[i].speed_mps=3.;
  ObservedVehicle other;other.x_m=0.;other.y_m=0.;other.vx_mps=4.;
  const auto fallback=node.connectedReturnFallback(command,*node.odometry_,0.,
      *node.base_reference_,active,DrivingMode::OVERTAKE,
      [](const auto &){return mppi::RejectReason::NONE;},{other},&profile);
  ASSERT_GE(fallback.trajectory.points.size(),3U);
  for(const auto &p:fallback.trajectory.points) {
    EXPECT_DOUBLE_EQ(p.pose.position.y,2.);EXPECT_FLOAT_EQ(p.longitudinal_velocity_mps,3.);
  }
}

TEST_F(NodeFallbackTest, AlongsideFallbackRecoversSpeedOnSamePath) {
  const auto command=prepareBrainScene(2.);
  auto active=command;active.corridor_side=1;
  for(auto &p:active.trajectory.points)p.pose.position.y=2.;
  ObservedVehicle other;other.y_m=0.;other.vx_mps=4.;
  const auto validator=[](const mppi::TemporaryReference &r) {
    for(std::size_t i=0;i<r.count;++i)
      if(r.points[i].speed_mps>2.)return mppi::RejectReason::COLLISION;
    return mppi::RejectReason::NONE;
  };
  const auto fallback=node.connectedReturnFallback(command,*node.odometry_,0.,
      *node.base_reference_,active,DrivingMode::OVERTAKE,validator,{other});
  EXPECT_EQ(fallback.reason,"mppi_brain:alongside_hold:speed_recovery");
  ASSERT_GE(fallback.trajectory.points.size(),3U);
  for(const auto &p:fallback.trajectory.points) {
    EXPECT_DOUBLE_EQ(p.pose.position.y,2.);
    EXPECT_GT(p.longitudinal_velocity_mps,0.);EXPECT_LE(p.longitudinal_velocity_mps,2.);
  }
}

TEST_F(NodeFallbackTest, BlockedAlongsideContinuationBrakesOnCurrentPathBeforeConsideringReturn) {
  const auto command=prepareBrainScene(2.);
  auto active=command;active.corridor_side=1;
  for(auto &p:active.trajectory.points)p.pose.position.y=2.;
  ObservedVehicle other;other.y_m=0.;other.vx_mps=4.;
  bool saw_reference_return=false;
  const auto validator=[&](const mppi::TemporaryReference &r) {
    for(std::size_t i=0;i<r.count;++i)
      if(std::abs(r.points[i].y_m-2.)>1e-9)saw_reference_return=true;
    for(std::size_t i=0;i<r.count;++i)
      if(r.points[i].speed_mps>0.)return mppi::RejectReason::COLLISION;
    return mppi::RejectReason::NONE;
  };
  const auto fallback=node.connectedReturnFallback(command,*node.odometry_,0.,
      *node.base_reference_,active,DrivingMode::OVERTAKE,validator,{other});
  EXPECT_FALSE(saw_reference_return);EXPECT_FALSE(node.isConnectedReturn(fallback));
  EXPECT_EQ(fallback.reason,"mppi_brain:alongside_hold:validated_braking_fallback");
  ASSERT_GE(fallback.trajectory.points.size(),3U);
  for(const auto &p:fallback.trajectory.points) {
    EXPECT_DOUBLE_EQ(p.pose.position.y,2.);EXPECT_FLOAT_EQ(p.longitudinal_velocity_mps,0.);
  }
}

TEST_F(NodeFallbackTest, ExhaustedAlongsidePathBrakesFromMeasuredPoseWithoutReferenceJump) {
  const auto command=prepareBrainScene(2.);
  auto active=command;active.corridor_side=1;
  for(auto &p:active.trajectory.points)p.pose.position.y=2.;
  node.odometry_->pose.pose.position.x=40.5;
  ObservedVehicle other;other.x_m=40.3;other.y_m=0.;other.vx_mps=4.;
  const auto fallback=node.connectedReturnFallback(command,*node.odometry_,0.,
      *node.base_reference_,active,DrivingMode::OVERTAKE,
      [](const auto &){return mppi::RejectReason::NONE;},{other});
  EXPECT_EQ(fallback.reason,"mppi_brain:alongside_hold:validated_braking_fallback");
  ASSERT_GE(fallback.trajectory.points.size(),3U);
  EXPECT_DOUBLE_EQ(fallback.trajectory.points.front().pose.position.x,40.5);
  for(const auto &p:fallback.trajectory.points) {
    EXPECT_DOUBLE_EQ(p.pose.position.y,2.);EXPECT_FLOAT_EQ(p.longitudinal_velocity_mps,0.);
  }
}

TEST_F(NodeFallbackTest, PassedOpponentDoesNotChangeOrdinaryReturnFallback) {
  const auto command=prepareBrainScene(2.);
  node.odometry_->pose.pose.position.x=10.;
  auto active=command;active.corridor_side=1;
  for(auto &p:active.trajectory.points)p.pose.position.y=2.;
  ObservedVehicle other;other.x_m=5.;other.y_m=0.;other.vx_mps=4.;
  const auto fallback=node.connectedReturnFallback(command,*node.odometry_,0.,
      *node.base_reference_,active,DrivingMode::OVERTAKE,
      [](const auto &){return mppi::RejectReason::NONE;},{other});
  EXPECT_TRUE(node.isConnectedReturn(fallback));EXPECT_EQ(fallback.corridor_side,0);
}

TEST_F(NodeFallbackTest, RejectedReturnBrakesOnCommittedGeometryWithoutReferenceJump) {
  auto command=prepareBrainScene(2.);
  auto active=command;active.corridor_side=1;
  for(auto &p:active.trajectory.points)p.pose.position.y=2.;
  const auto validator=[](const mppi::TemporaryReference &r) {
    bool stopped=true,on_active=true;
    for(std::size_t i=0;i<r.count;++i) {
      stopped=stopped && r.points[i].speed_mps==0.;
      on_active=on_active && std::abs(r.points[i].y_m-2.)<1e-9;
    }
    return stopped && on_active ? mppi::RejectReason::NONE : mppi::RejectReason::WALL;
  };
  const auto returned=node.connectedReturnFallback(command,*node.odometry_,0.,
      *node.base_reference_,active,DrivingMode::OVERTAKE,validator);
  EXPECT_FALSE(node.isConnectedReturn(returned));EXPECT_EQ(returned.corridor_side,1);
  EXPECT_EQ(returned.reason,"mppi_brain:return_unavailable:validated_braking_fallback");
  ASSERT_GT(returned.trajectory.points.size(),2U);
  for(const auto &p:returned.trajectory.points) {
    EXPECT_NEAR(p.pose.position.y,2.,1e-9);EXPECT_FLOAT_EQ(p.longitudinal_velocity_mps,0.);
  }
}

TEST_F(NodeFallbackTest, FollowExecutesPreparationSpeedWhileConnectionSearchIsDue) {
  auto command=prepareBrainScene();
  node.config_.longitudinal_planning_enabled=true;
  node.config_.collision_only_rejection=true;
  node.config_.speed_proportional_gain=3.;node.config_.maximum_acceleration_mps2=2.;
  node.config_.maximum_deceleration_mps2=2.;node.config_.horizon_steps=40;
  auto inputs=node.captureBrainInputs();
  auto context=node.executionContext(inputs,*node.odometry_);
  ASSERT_TRUE(context);
  for(std::size_t i=0;i<context->base_reference_count;++i) context->base_reference[i].speed_mps=9.;
  context->rollout_constraint_validator=[](const auto &,const auto &b) {
    return b.x_m>9. ? mppi::RejectReason::WALL : mppi::RejectReason::NONE;
  };
  auto preparation=std::make_shared<mppi::PassingPreparationPlan>();
  preparation->world=context->world_reference;preparation->epoch_sec=context->stamp_sec;
  preparation->lateral_start_time_sec=10.;
  for(int i=0;i<=300;++i) preparation->schedule.push_back({i*.05,3.*i*.05,3.,0.});
  context->passing_preparation=preparation;
  ASSERT_TRUE(mppi::preparationSearchDue(*preparation,context->stamp_sec,.1));
  const auto prepared=node.planBrainFollowSpeed(command,*context);
  EXPECT_EQ(prepared.reason,"mppi_brain:preparation_speed");
  EXPECT_LT(prepared.trajectory.points.front().longitudinal_velocity_mps,4.);
  EXPECT_EQ(prepared.trajectory.points.size(),command.trajectory.points.size());
  for(std::size_t i=0;i<command.trajectory.points.size();++i)
    EXPECT_EQ(prepared.trajectory.points[i].pose,command.trajectory.points[i].pose);
  context->passing_preparation.reset();
  EXPECT_EQ(node.planBrainFollowSpeed(command,*context).reason,"mppi_brain:longitudinal_follow");
}

TEST_F(NodeFallbackTest, ReferenceIndexSharesImmutableGeometryAndKeepsOldSnapshots) {
  stopBrainWorker();
  auto first=std::make_shared<Trajectory>(desired().trajectory);
  const auto old=node.sharedReferencePoseIndex(first);
  ASSERT_TRUE(old);
  EXPECT_EQ(old,node.sharedReferencePoseIndex(first));
  auto speed_only=std::make_shared<Trajectory>(*first);
  speed_only->points[0].longitudinal_velocity_mps=0.;
  EXPECT_EQ(old,node.sharedReferencePoseIndex(speed_only));
  auto changed=std::make_shared<Trajectory>(*speed_only);
  for(auto &p:changed->points)p.pose.position.y+=2.;
  const auto newer=node.sharedReferencePoseIndex(changed);
  ASSERT_NE(old,newer);
  ASSERT_TRUE(old->pose(5.,0.));ASSERT_TRUE(newer->pose(5.,0.));
  EXPECT_DOUBLE_EQ(old->pose(5.,0.)->at(1),0.);
  EXPECT_DOUBLE_EQ(newer->pose(5.,0.)->at(1),2.);
  auto parallel=std::async(std::launch::async,[&]{return node.sharedReferencePoseIndex(changed);});
  EXPECT_EQ(newer,parallel.get());
}

TEST_F(NodeFallbackTest, PartialPreparationStartsFastLeaderSearchBeforeFiveMetres) {
  auto command=prepareBrainScene();
  node.passing_preparation_enabled_=node.leader_lap_prediction_enabled_=true;
  node.config_.longitudinal_planning_enabled=true;
  node.config_.collision_only_rejection=true;node.config_.awsim_vehicle_response_enabled=true;
  node.gentle_lateral_acceleration_mps2_=1.5;
  node.config_.maximum_lateral_acceleration_mps2=6.;
  node.config_.maximum_acceleration_mps2=2.;node.config_.speed_proportional_gain=3.;
  node.brain_overtake_speed_mps_=node.brain_cruise_speed_mps_=35./3.6;
  node.base_reference_->points.clear();
  constexpr double radius=40.,gap=12.,leader_speed=8.4;
  for(int i=0;i<=720;++i) {
    const double angle=2.*M_PI*i/720.;
    autoware_auto_planning_msgs::msg::TrajectoryPoint p;
    p.pose.position.x=radius*std::cos(angle);p.pose.position.y=radius*std::sin(angle);
    p.pose.orientation=quaternionFromYaw(angle+M_PI/2.);p.longitudinal_velocity_mps=35./3.6;
    node.base_reference_->points.push_back(p);
  }
  node.odometry_->pose.pose.position.x=radius;node.odometry_->pose.pose.position.y=0.;
  node.odometry_->pose.pose.orientation=quaternionFromYaw(M_PI/2.);
  node.odometry_->twist.twist.linear.x=8.5;
  const double epoch=rclcpp::Time(node.odometry_->header.stamp).seconds();
  node.leader_vehicle_id_="d3";node.leader_receive_sec_=epoch;
  ObservedVehicle target;target.id="d3";target.x_m=radius*std::cos(gap/radius);
  target.y_m=radius*std::sin(gap/radius);target.vx_mps=-leader_speed*std::sin(gap/radius);
  target.vy_mps=leader_speed*std::cos(gap/radius);target.stamp_sec=target.receive_sec=epoch;
  auto history=std::make_shared<std::vector<opponent_prediction::PositionObservation>>();
  for(int i=-4000;i<=0;++i) {
    const double t=i*.02,s=gap+leader_speed*t;
    history->push_back({epoch+t,radius*std::cos(s/radius),radius*std::sin(s/radius),s});
  }
  target.history=history;node.observed_vehicles_[target.id]=target;
  node.receiveBrainCommand(command);
  ASSERT_TRUE(node.latest_preparation_);
  ASSERT_FALSE(node.latest_preparation_->complete_pass);
  ASSERT_TRUE(node.pending_batch_);
  ASSERT_EQ(node.pending_batch_->candidate_count,2U);
  EXPECT_EQ(node.pending_batch_->maneuver,DrivingMode::OVERTAKE);
  for(const auto &candidate:node.pending_batch_->candidates) {
    EXPECT_TRUE(candidate.request.connect_to_wall_line);
    EXPECT_EQ(candidate.request.precomputed_wall_lines,node.latest_preparation_->wall_lines);
    EXPECT_EQ(candidate.request.passing_preparation,node.latest_preparation_);
    EXPECT_EQ(candidate.maneuver,DrivingMode::OVERTAKE);
    EXPECT_DOUBLE_EQ(candidate.request.nominal.d_pass_m,candidate.request.pass_profile_scale_m);
    EXPECT_DOUBLE_EQ(candidate.request.bounds.minimum.d_pass_m,candidate.request.bounds.maximum.d_pass_m);
  }
  EXPECT_EQ(node.driving_fsm_.mode(),DrivingMode::FREE_RUN);
  auto inputs=node.captureBrainInputs();
  ASSERT_EQ(inputs.opponents.size(),1U);
  EXPECT_TRUE(inputs.opponents.front().prior_lap_match.usable);
  auto owned=std::make_shared<mppi::PassingPreparationPlan>(*node.latest_preparation_);
  owned->side=-1;owned->connection=std::make_shared<const mppi::TemporaryReference>();
  inputs.active_preparation=owned;inputs.active_command=command;
  inputs.driving_fsm.acceptLateral(DrivingMode::OVERTAKE);
  auto context=node.executionContext(inputs,*inputs.odometry);ASSERT_TRUE(context);
  context->leader_passing_road.reset();
  node.updatePassingPreparation(inputs,*context);
  EXPECT_EQ(inputs.preparation,owned);
  EXPECT_EQ(context->passing_preparation,owned);
  EXPECT_EQ(node.latest_preparation_,owned);
  node.active_preparation_=owned;
  node.clearBrainExecution();
  EXPECT_FALSE(node.active_preparation_);EXPECT_FALSE(node.latest_preparation_);
}

ReferencePoseIndex displayTestReference(const std::vector<double> &stations) {
  return ReferencePoseIndex(stations,[](double s){return std::array<double,2>{s,0.};});
}

TEST(PassingPointDisplayTest, DeadlineUsesMetresWithUnevenWaypointSpacing) {
  PassingPointDisplayConfig config;
  config.straight_start_xy={7.,0.};config.corner_entry_xy={23.,0.};
  const auto areas=passingPointDisplayAreas(displayTestReference({0.,7.,11.,23.,31.}),31.,false,config);
  ASSERT_TRUE(areas);ASSERT_EQ(areas->allowed.size(),1U);
  EXPECT_DOUBLE_EQ(areas->allowed[0].first,7.);
  EXPECT_DOUBLE_EQ(areas->allowed[0].second,18.);
  ASSERT_EQ(areas->forbidden.size(),2U);
  EXPECT_EQ(areas->forbidden[0],std::make_pair(0.,7.));
  EXPECT_EQ(areas->forbidden[1],std::make_pair(18.,31.));
}

TEST(PassingPointDisplayTest, ClosedCourseStraightCrossesSeamWithoutBlockingItsMiddle) {
  PassingPointDisplayConfig config;
  config.straight_start_xy={30.,0.};config.corner_entry_xy={10.,0.};
  const auto world=displayTestReference({0.,10.,20.,30.,40.});
  const auto areas=passingPointDisplayAreas(world,40.,true,config);
  ASSERT_TRUE(areas);ASSERT_EQ(areas->allowed.size(),2U);
  EXPECT_EQ(areas->allowed[0],std::make_pair(0.,5.));
  EXPECT_EQ(areas->allowed[1],std::make_pair(30.,40.));
  ASSERT_EQ(areas->forbidden.size(),1U);
  EXPECT_EQ(areas->forbidden[0],std::make_pair(5.,30.));
  EXPECT_FALSE(passingPointDisplayAreas(world,40.,false,config));
}

TEST(PassingPointDisplayTest, ShortStraightHasNoSpaceBeforeDeadline) {
  PassingPointDisplayConfig config;
  config.straight_start_xy={10.,0.};config.corner_entry_xy={13.,0.};
  const auto areas=passingPointDisplayAreas(displayTestReference({0.,10.,13.,20.}),20.,false,config);
  ASSERT_TRUE(areas);EXPECT_TRUE(areas->allowed.empty());
  ASSERT_EQ(areas->forbidden.size(),1U);
  EXPECT_EQ(areas->forbidden[0],std::make_pair(0.,20.));
}

TEST(PassingPointDisplayTest, PlacementMembershipUsesSameMetricMarginAcrossLaps) {
  PassingPointDisplayConfig config;config.straight_start_xy={30.,0.};config.corner_entry_xy={10.,0.};
  const auto areas=passingPointDisplayAreas(displayTestReference({0.,10.,20.,30.,40.}),40.,true,config);
  ASSERT_TRUE(areas);
  for(double s:{0.,5.,30.,40.,45.,70.,-5.}) EXPECT_TRUE(areas->contains(s))<<s;
  for(double s:{5.01,20.,29.99,45.01,60.}) EXPECT_FALSE(areas->contains(s))<<s;
  EXPECT_FALSE(areas->contains(NAN));
  config.straight_start_xy.clear();config.corner_entry_xy.clear();
  const auto empty=passingPointDisplayAreas(displayTestReference({0.,40.}),40.,false,config);
  ASSERT_TRUE(empty);EXPECT_FALSE(empty->contains(20.));
  ASSERT_EQ(empty->forbidden.size(),1U);EXPECT_EQ(empty->forbidden[0],std::make_pair(0.,40.));
}

TEST_F(NodeFallbackTest, PlacementRestrictionReachesRequestsIndependentlyOfRvizVisibility) {
  prepareBrainScene();
  node.passing_point_display_enabled_=false;node.passing_point_areas_enabled_=true;
  node.passing_point_display_config_.straight_start_xy={5.,0.};
  node.passing_point_display_config_.corner_entry_xy={20.,0.};
  const auto inputs=node.captureBrainInputs();
  auto context=node.executionContext(inputs,*inputs.odometry);ASSERT_TRUE(context);
  ASSERT_TRUE(context->passing_point_areas);
  EXPECT_TRUE(context->passing_point_areas->contains(10.));
  EXPECT_FALSE(context->passing_point_areas->contains(16.));
  EXPECT_FALSE(node.passing_point_display_published_);
  node.passing_point_areas_enabled_=false;
  context=node.executionContext(inputs,*inputs.odometry);ASSERT_TRUE(context);
  EXPECT_FALSE(context->passing_point_areas);
}

TEST_F(NodeFallbackTest, ForbiddenAreaIsAvailableWithoutOpponentAndHasMetricBoundary) {
  prepareBrainScene();
  const auto inputs=node.captureBrainInputs();
  ASSERT_TRUE(inputs.wall_lines);ASSERT_TRUE(inputs.opponents.empty());
  PassingPointDisplayConfig config;
  config.straight_start_xy={5.,0.};config.corner_entry_xy={20.,0.};
  const auto markers=makePassingPointForbiddenMarkers(inputs.wall_lines.get(),config,std_msgs::msg::Header{});
  ASSERT_EQ(markers.markers.size(),4U);
  EXPECT_EQ(markers.markers[0].action,visualization_msgs::msg::Marker::DELETEALL);
  const auto &fill=markers.markers[1];
  ASSERT_EQ(fill.type,visualization_msgs::msg::Marker::TRIANGLE_LIST);
  EXPECT_EQ(fill.points.size()%6,0U);EXPECT_GT(fill.color.a,0.);EXPECT_LT(fill.color.a,1.);
  for (std::size_t i=0;i<fill.points.size();i+=6) {
    const double middle=.5*(fill.points[i].x+fill.points[i+2].x);
    EXPECT_TRUE(middle<=5. || middle>=15.);
  }
  const auto &boundary=markers.markers.back();
  ASSERT_EQ(boundary.points.size(),2U);
  EXPECT_NEAR(boundary.points[0].x,15.,1e-9);
  EXPECT_NEAR(boundary.points[1].x,15.,1e-9);
  EXPECT_GT(boundary.points[0].y,boundary.points[1].y);
  EXPECT_EQ(makePassingPointForbiddenMarkers(nullptr,config,std_msgs::msg::Header{}).markers.size(),1U);
  config.corner_entry_xy={20.};
  EXPECT_EQ(makePassingPointForbiddenMarkers(inputs.wall_lines.get(),config,std_msgs::msg::Header{}).markers.size(),1U);
}

TEST_F(NodeFallbackTest, ForbiddenAreaCacheFollowsReferenceAndClearsMissingGeometry) {
  prepareBrainScene();
  node.passing_point_display_enabled_=true;
  node.passing_point_display_config_.straight_start_xy={5.,0.};
  node.passing_point_display_config_.corner_entry_xy={20.,0.};
  auto inputs=node.captureBrainInputs();
  EXPECT_TRUE(node.passing_point_display_published_);
  EXPECT_EQ(node.displayed_passing_point_lines_,inputs.wall_lines);
  auto reference=std::make_shared<Trajectory>(*node.base_reference_);
  for (auto &p:reference->points) p.pose.position.y+=.25;
  node.base_reference_=reference;
  const auto updated=node.captureBrainInputs();
  EXPECT_NE(updated.wall_lines,inputs.wall_lines);
  EXPECT_EQ(node.displayed_passing_point_lines_,updated.wall_lines);
  node.base_reference_.reset();node.captureBrainInputs();
  EXPECT_FALSE(node.displayed_passing_point_lines_);
}

TEST(PassingPointDisplayTest, FixedAnchorsAreInvariantToReferenceResampling) {
  PassingPointDisplayConfig config;
  config.straight_start_xy={7.3,0.};config.corner_entry_xy={23.4,0.};
  std::vector<double> dense;
  for (int i=0;i<=310;++i) dense.push_back(i*.1);
  const auto coarse=passingPointDisplayAreas(displayTestReference({0.,7.,11.,23.,31.}),31.,false,config);
  const auto fine=passingPointDisplayAreas(displayTestReference(dense),31.,false,config);
  ASSERT_TRUE(coarse);ASSERT_TRUE(fine);
  ASSERT_EQ(coarse->allowed.size(),1U);ASSERT_EQ(fine->allowed.size(),1U);
  EXPECT_NEAR(coarse->allowed[0].first,fine->allowed[0].first,1e-10);
  EXPECT_NEAR(coarse->deadlines[0],fine->deadlines[0],1e-10);
  EXPECT_NEAR(fine->deadlines[0],18.4,1e-10);
}

TEST_F(NodeFallbackTest, WallLinesExistBeforeOpponentAndReuseAcrossEgoAndSpeedUpdates) {
  prepareBrainScene();
  const auto first=node.captureBrainInputs();
  ASSERT_TRUE(first.wall_lines);EXPECT_TRUE(first.opponents.empty());
  EXPECT_FALSE(first.preparation);
  ASSERT_TRUE(first.wall_lines->at(10.,1));ASSERT_TRUE(first.wall_lines->at(10.,-1));
  EXPECT_GT(first.wall_lines->at(10.,1)->at(1),0.);
  EXPECT_LT(first.wall_lines->at(10.,-1)->at(1),0.);
  node.odometry_->pose.pose.position.x=7.;node.odometry_->twist.twist.linear.x=8.;
  auto reference=std::make_shared<Trajectory>(*node.base_reference_);
  reference->points[0].longitudinal_velocity_mps=2.;node.base_reference_=reference;
  auto same_map=std::make_shared<nav_msgs::msg::OccupancyGrid>(*node.wall_map_);
  same_map->header.stamp.sec+=1;node.wall_map_=same_map;
  const auto moved=node.captureBrainInputs();
  EXPECT_EQ(moved.wall_lines,first.wall_lines);
  EXPECT_EQ(moved.wall_lines->at(10.,1),first.wall_lines->at(10.,1));
  auto changed=std::make_shared<nav_msgs::msg::OccupancyGrid>(*same_map);
  changed->data[0]=100;node.wall_map_=changed;
  node.wall_index_=std::make_shared<OccupancyGridWallIndex>(changed);
  const auto new_map=node.captureBrainInputs();
  ASSERT_TRUE(new_map.wall_lines);EXPECT_NE(new_map.wall_lines,first.wall_lines);
  auto geometry=std::make_shared<Trajectory>(*reference);
  for(auto &p:geometry->points) p.pose.position.y+=.5;
  node.base_reference_=geometry;
  const auto new_route=node.captureBrainInputs();
  EXPECT_NE(new_route.wall_lines,new_map.wall_lines);
  EXPECT_EQ(first.wall_lines->at(10.,1),moved.wall_lines->at(10.,1));
}

TEST_F(NodeFallbackTest, WallLineWrapPreservesBothSidesAtLapSeam) {
  prepareBrainScene();
  auto route=std::make_shared<Trajectory>();
  for(int i=0;i<=360;++i) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint p;
    p.pose.position.x=20.*std::cos(i*M_PI/180.);
    p.pose.position.y=20.*std::sin(i*M_PI/180.);route->points.push_back(p);
  }
  node.base_reference_=route;
  const auto inputs=node.captureBrainInputs();
  ASSERT_TRUE(inputs.wall_lines);ASSERT_TRUE(inputs.wall_lines->closed);
  for(int side:{-1,1}) {
    const auto &lines=*inputs.wall_lines;
    ASSERT_TRUE(lines.at(-.01,side));ASSERT_TRUE(lines.at(.01,side));
    const auto a=*lines.at(-.01,side),b=*lines.at(.01,side);
    EXPECT_LT(std::hypot(a[0]-b[0],a[1]-b[1]),.03);
    EXPECT_EQ(lines.at(0.,side),lines.at(lines.length_m,side));
  }
}

TEST_F(NodeFallbackTest, PreparationMarkersUseAdoptedPlanAndExpire) {
  prepareBrainScene();
  mppi::PassingPreparationPlan plan;plan.id=17;plan.revision=31;plan.target_id="d3";
  plan.world=node.sharedReferencePoseIndex(node.base_reference_);
  plan.epoch_sec=10.;plan.origin_station_m=0.;plan.entry_station_m=10.;plan.exit_station_m=30.;
  plan.acceleration_start_station_m=3.;plan.lateral_start_station_m=5.;
  plan.pass_station_m=20.;plan.pass_time_sec=4.;
  std_msgs::msg::Header header;header.stamp.sec=11;
  const auto markers=makePreparationPlanMarkers(&plan,header,true);
  ASSERT_EQ(markers.markers.size(),6U);
  EXPECT_EQ(markers.markers[4].text,"ACTIVE d3 plan=17/31\npass +3.0 s");
  EXPECT_DOUBLE_EQ(markers.markers[2].pose.position.x,3.);
  EXPECT_DOUBLE_EQ(markers.markers[5].pose.position.x,5.);
  header.stamp.sec=15;
  EXPECT_EQ(makePreparationPlanMarkers(&plan,header,true).markers.size(),1U);
}

TEST_F(NodeFallbackTest, PartialPreparationMarkersShowPreparationAndItsOwnDeadline) {
  prepareBrainScene();
  mppi::PassingPreparationPlan plan;plan.id=17;plan.revision=31;plan.target_id="d3";
  plan.world=node.sharedReferencePoseIndex(node.base_reference_);
  plan.epoch_sec=10.;plan.entry_station_m=10.;plan.exit_station_m=30.;
  plan.acceleration_start_station_m=3.;plan.lateral_start_station_m=5.;
  plan.complete_pass=false;plan.pass_time_sec=plan.pass_station_m=-1.;
  plan.schedule={{0.,0.,5.,0.},{5.,29.,7.,0.}};
  std_msgs::msg::Header header;header.stamp.sec=11;
  const auto markers=makePreparationPlanMarkers(&plan,header,true);
  ASSERT_EQ(markers.markers.size(),6U);
  EXPECT_EQ(markers.markers[4].text,"ACTIVE d3 plan=17/31\nprepare +4.0 s");
  EXPECT_DOUBLE_EQ(markers.markers[3].pose.position.x,29.);
  header.stamp.sec=16;
  EXPECT_EQ(makePreparationPlanMarkers(&plan,header,true).markers.size(),1U);
}

TEST_F(NodeFallbackTest, PassingMarkersUseMeasuredOriginAndSelectedCompletionDistance) {
  prepareBrainScene();
  mppi::PlanRequest request;
  request.world_reference = node.sharedReferencePoseIndex(node.base_reference_);
  request.ego.x_m = 5.; request.ego.y_m = 2.;
  request.leader_preparation_m = 2.;
  request.leader_passing_entry_m = 10.; request.leader_passing_exit_m = 20.;
  request.leader_pass_distance_m = 15.; request.leader_pass_time_sec = 3.5;
  auto header = node.odometry_->header; header.frame_id = "track_map";
  const auto markers = makePassingOpportunityMarkers(&request, header, "d3");
  ASSERT_EQ(markers.markers.size(), 5U);
  EXPECT_EQ(markers.markers[0].action, visualization_msgs::msg::Marker::DELETEALL);
  const auto &interval = markers.markers[1];
  EXPECT_DOUBLE_EQ(interval.points.front().x, 15.);
  EXPECT_DOUBLE_EQ(interval.points.back().x, 25.);
  EXPECT_DOUBLE_EQ(markers.markers[2].pose.position.x, 7.);
  EXPECT_DOUBLE_EQ(markers.markers[3].pose.position.x, 20.);
  EXPECT_DOUBLE_EQ(markers.markers[3].pose.position.y, 0.);
  EXPECT_EQ(markers.markers[3].header.frame_id, "track_map");
  EXPECT_EQ(markers.markers[3].header.stamp, header.stamp);
  EXPECT_EQ(markers.markers[4].text, "PASS CANDIDATE d3\n+3.5 s / 15.0 m ahead");
  EXPECT_EQ(markers.markers[3].lifetime.nanosec, 500000000U);
}

TEST_F(NodeFallbackTest, MissingPassingOpportunityClearsItsMarkers) {
  prepareBrainScene();
  mppi::PlanRequest request;
  request.world_reference = node.sharedReferencePoseIndex(node.base_reference_);
  request.leader_passing_entry_m = 10.; request.leader_passing_exit_m = 20.;
  request.leader_preparation_m = 2.; request.leader_pass_time_sec = 3.5;
  for (const auto *input : std::array<const mppi::PlanRequest *, 2>{nullptr, &request}) {
    const auto markers = makePassingOpportunityMarkers(input, std_msgs::msg::Header{}, "d3");
    ASSERT_EQ(markers.markers.size(), 1U);
    EXPECT_EQ(markers.markers[0].action, visualization_msgs::msg::Marker::DELETEALL);
    EXPECT_EQ(markers.markers[0].header.frame_id, "map");
  }
}

TEST_F(NodeFallbackTest, HighwaySpeedOpponentCanBeSelectedAndFollowed) {
  const auto base=desired();
  nav_msgs::msg::Odometry ego;
  ego.pose.pose.position.x=5.;ego.pose.pose.orientation.w=1.;
  ego.twist.twist.linear.x=8.;
  BrainInputs inputs;
  ObservedVehicle lead;
  lead.id="fast_lead";lead.x_m=5.+2.2+3.;lead.vx_mps=6.9;
  inputs.opponents.push_back(lead);
  const auto target=node.selectFollowVehicle(base.trajectory,base.trajectory,ego,inputs);
  ASSERT_TRUE(target);
  EXPECT_EQ(target->vehicle.id,"fast_lead");
  EXPECT_NEAR(node.makeBrainFollowCommand(base,target,node.driving_fsm_).trajectory.points.front().longitudinal_velocity_mps,
              10.,1e-6);
  inputs.opponents.front().vx_mps=11.;
  ASSERT_TRUE(node.selectFollowVehicle(base.trajectory,base.trajectory,ego,inputs));
}

TEST_F(NodeFallbackTest, RotationSnapshotUsesVehicleEpochAndReachesLiveValidator) {
  prepareBrainScene();
  node.odometry_->header.stamp.sec=10;
  node.odometry_->header.stamp.nanosec=0;
  simple_pure_pursuit::RotationControllerParameters parameters;
  parameters.rotation_gate_enabled=true;
  simple_pure_pursuit::RotationControllerState state;
  state.active=true;state.started_sec=9.8;state.filtered_yaw_acceleration_radps2=.4;
  auto message=simple_pure_pursuit::makeRotationPredictionMessage(parameters,state,.03,true);
  message.header.stamp.sec=9;message.header.stamp.nanosec=900000000;
  auto snapshot=simple_pure_pursuit::rotationPredictionSnapshot(message);
  node.rotation_prediction_history_.push_back(snapshot);
  auto future=snapshot;future.stamp_sec=10.1;future.state.active=false;
  node.rotation_prediction_history_.push_back(future);
  const auto inputs=node.captureBrainInputs();
  ASSERT_TRUE(inputs.rotation_prediction.valid);
  EXPECT_DOUBLE_EQ(inputs.rotation_prediction.stamp_sec,9.9);
  EXPECT_TRUE(inputs.rotation_prediction.state.active);
  EXPECT_DOUBLE_EQ(inputs.rotation_prediction.slip_angle_rad,.03);
  const auto context=node.executionContext(inputs,*inputs.odometry);
  ASSERT_TRUE(context);
  EXPECT_TRUE(context->rotation_prediction.state.active);
  EXPECT_DOUBLE_EQ(context->rotation_prediction.state.filtered_yaw_acceleration_radps2,.4);
  node.odometry_->header.stamp.sec=11;
  EXPECT_FALSE(node.captureBrainInputs().rotation_prediction.valid);
}

TEST_F(NodeFallbackTest, SuppliedTailSurvivesHoldWithoutExtendingPredictionTime) {
  auto global=std::make_shared<Trajectory>(desired().trajectory);
  for(int i=80;i<=160;++i) {
    auto p=global->points.front();p.pose.position.x=i*.5;global->points.push_back(p);
  }
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.resolution=.1F;map->info.width=1000;map->info.height=400;
  map->info.origin.position.x=-10.;map->info.origin.position.y=-20.;
  map->info.origin.orientation.w=1.;map->data.assign(400000,0);
  BrainInputs inputs;inputs.base_reference=global;inputs.wall_map=map;
  inputs.wall_index=std::make_shared<OccupancyGridWallIndex>(map);
  nav_msgs::msg::Odometry odom;odom.header.stamp.sec=122;
  odom.pose.pose.orientation.w=1.;odom.twist.twist.linear.x=10.;
  node.reference_supply_sec_=0.;
  const auto before=node.executionContext(inputs,odom);ASSERT_TRUE(before);
  node.reference_supply_sec_=1.;
  const auto after=node.executionContext(inputs,odom);ASSERT_TRUE(after);
  EXPECT_EQ(before->horizon_steps_override,after->horizon_steps_override);
  EXPECT_NEAR(after->base_reference[after->base_reference_count-1].s_m,40.,1e-8);
  FollowVehicle target;target.vehicle.id="d2";target.relative_s_m=15.;
  target.vehicle.x_m=15.;target.vehicle.vx_mps=4.;target.longitudinal_speed_mps=4.;
  target.projection=node.projectOnTrajectory(*global,15.,0.);
  const auto work=node.makeBrainWork(desired(),odom,inputs,1);ASSERT_TRUE(work);
  EXPECT_EQ(work->request.horizon_steps_override,before->horizon_steps_override);
  EXPECT_NEAR(work->actual_horizon_m,40.,1e-8);
  Command accepted=desired();accepted.trajectory.points=work->command.trajectory.points;
  odom.pose.pose.position.x=8.;
  const auto hold=node.refreshedBrainHold(accepted,accepted,odom,*global);
  EXPECT_NEAR(hold.remaining_arc_m,32.,1e-5);
  EXPECT_GT(hold.remaining_arc_m,node.localDistance(10.));
  EXPECT_NEAR(hold.trajectory.points.back().pose.position.x,40.,1e-8);
}
TEST_F(NodeFallbackTest, EarlierPassEntryRetainsGentleNominalAndMerge) {
  prepareBrainScene();
  auto inputs=node.captureBrainInputs();
  auto ego=*inputs.odometry;
  ego.twist.twist.linear.x=8.0;
  const auto passing=node.makeBrainWork(desired(),ego,inputs,1);
  const auto merging=node.makeBrainWork(desired(),ego,inputs,1,mppi::Phase::MERGE);
  ASSERT_TRUE(passing);ASSERT_TRUE(merging);
  EXPECT_DOUBLE_EQ(passing->request.bounds.minimum.l_out_m,12.0);
  EXPECT_GE(passing->request.nominal.l_out_m,15.0);
  EXPECT_GT(passing->request.bounds.maximum.l_out_m,passing->request.nominal.l_out_m);
  EXPECT_DOUBLE_EQ(merging->request.bounds.minimum.l_out_m,15.0);
  EXPECT_TRUE(passing->request.path_constraint_validator);
  EXPECT_TRUE(passing->request.rollout_constraint_validator);
  EXPECT_DOUBLE_EQ(passing->request.minimum_speed_mps,0.0);
  ego.twist.twist.linear.x=1.0;
  const auto slow=node.makeBrainWork(desired(),ego,inputs,1);
  ASSERT_TRUE(slow);
  EXPECT_DOUBLE_EQ(slow->request.bounds.minimum.l_out_m,4.0);
}

TEST_F(NodeFallbackTest, FrontMergeDenseReferenceSuppliesWholeManeuverWithinCapacity) {
  stopBrainWorker();node.front_merge_attack_enabled_=true;node.passing_preparation_enabled_=true;
  auto global=std::make_shared<Trajectory>();
  for(int i=0;i<=2000;++i) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint p;
    p.pose.position.x=.1*i;p.pose.orientation.w=1.;p.longitudinal_velocity_mps=10.;
    global->points.push_back(p);
  }
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.resolution=.5;map->info.width=500;map->info.height=100;
  map->info.origin.position.x=-10.;map->info.origin.position.y=-25.;
  map->info.origin.orientation.w=1.;map->data.assign(50000,0);
  BrainInputs inputs;inputs.base_reference=global;inputs.wall_map=map;
  inputs.wall_index=std::make_shared<OccupancyGridWallIndex>(map);
  auto lines=std::make_shared<PrecomputedWallLines>();
  lines->world=node.sharedReferencePoseIndex(global);lines->length_m=200.;
  for(int i=0;i<=200;++i)lines->samples.push_back({double(i),{{{double(i),3.},{double(i),-3.}}}});
  inputs.wall_lines=lines;
  nav_msgs::msg::Odometry ego;ego.pose.pose.orientation.w=1.;ego.twist.twist.linear.x=8.;
  inputs.odometry=ego;inputs.leader_vehicle_id="fast";
  ObservedVehicle target;target.id="fast";target.x_m=8.;target.vx_mps=7.;inputs.opponents.push_back(target);
  auto plan=std::make_shared<mppi::PassingPreparationPlan>();
  plan->world=lines->world;plan->wall_lines=lines;plan->target_id="fast";plan->front_merge=true;
  plan->entry_station_m=20.;
  plan->merge_start_station_m=60.;plan->merge_end_station_m=90.;plan->exit_station_m=120.;
  plan->pass_time_sec=10.;inputs.preparation=plan;
  EXPECT_FALSE(node.makeBrainWork(desired(),ego,inputs,1,mppi::Phase::OVERTAKE,DrivingMode::OVERTAKE));
  plan->epoch_sec=122.;
  const auto work=node.makeBrainWork(desired(),ego,inputs,1,mppi::Phase::OVERTAKE,DrivingMode::OVERTAKE);
  ASSERT_TRUE(work);EXPECT_TRUE(work->request.front_merge_attack);
  EXPECT_DOUBLE_EQ(work->request.nominal.l_out_m,20.);
  EXPECT_DOUBLE_EQ(work->request.nominal.l_out_m+work->request.nominal.l_hold_m,60.);
  EXPECT_DOUBLE_EQ(work->request.bounds.minimum.l_hold_m,work->request.bounds.maximum.l_hold_m);
  EXPECT_DOUBLE_EQ(work->request.bounds.minimum.d_pass_m,work->request.pass_profile_scale_m);
  EXPECT_DOUBLE_EQ(work->request.bounds.maximum.d_pass_m,work->request.pass_profile_scale_m);
  auto entry_request=work->request;
  node.gentle_lateral_acceleration_mps2_=.1;
  node.applyGentleLateralEntry(entry_request);
  EXPECT_DOUBLE_EQ(entry_request.nominal.l_out_m,20.);
  EXPECT_LE(work->request.base_reference_count,mppi::kMaximumReferencePoints);
  EXPECT_FALSE(work->reference_capacity_limited);
  EXPECT_NEAR(work->actual_horizon_m,120.+node.suppliedDistance(8.),1e-8);
  EXPECT_EQ(work->request.base_reference_count,work->command.trajectory.points.size());
  EXPECT_GT(work->actual_horizon_m-plan->merge_end_station_m,node.localDistance(8.));
  EXPECT_GE(work->actual_horizon_m-plan->exit_station_m,node.suppliedDistance(8.)-1e-8);
  node.multiple_passing_points_enabled_=true;
  std::vector<WorkItem> options;
  for(int option=0;option<3;++option) {
    auto later=std::make_shared<mppi::PassingPreparationPlan>(*plan);
    later->id=100+option;later->pass_station_m=later->merge_start_station_m+=option*10.;
    later->merge_end_station_m+=option*10.;later->exit_station_m+=option*10.;
    later->prior_lap_guidance=true;later->side=0;inputs.preparation=later;
    for(int side:{1,-1}) {
      const auto candidate=node.makeBrainWork(desired(),ego,inputs,side,mppi::Phase::OVERTAKE,DrivingMode::OVERTAKE);
      ASSERT_TRUE(candidate);ASSERT_TRUE(candidate->request.compare_passing_points);
      const auto evaluation_request=node.executionRequestForCandidate(work->request,candidate->request);
      EXPECT_EQ(evaluation_request.passing_preparation,later);
      EXPECT_DOUBLE_EQ(evaluation_request.leader_passing_exit_m,candidate->request.leader_passing_exit_m);
      EXPECT_EQ(evaluation_request.side,side);
      EXPECT_TRUE(evaluation_request.front_merge_attack);
      options.push_back(*candidate);
    }
  }
  ASSERT_EQ(options.size(),6U);EXPECT_LE(options.size(),mppi::kMaximumBatchCandidateCount);
  inputs.preparation=plan;node.multiple_passing_points_enabled_=false;
  plan->prior_lap_guidance=true;plan->side=-1;plan->epoch_sec=122.;
  EXPECT_FALSE(node.makeBrainWork(desired(),ego,inputs,1,mppi::Phase::OVERTAKE,DrivingMode::OVERTAKE));
  const auto locked=node.makeBrainWork(desired(),ego,inputs,-1,mppi::Phase::OVERTAKE,DrivingMode::OVERTAKE);
  ASSERT_TRUE(locked);EXPECT_EQ(locked->request.side,-1);
  EXPECT_LT(locked->request.nominal.d_pass_m,0.);
  EXPECT_DOUBLE_EQ(locked->request.nominal.d_pass_m,locked->request.pass_profile_scale_m);
  EXPECT_EQ(locked->request.passing_preparation,plan);
  plan->prior_lap_guidance=false;
  plan->side=1;plan->connection=std::make_shared<mppi::TemporaryReference>();
  inputs.active_preparation=plan;inputs.active_command=desired();inputs.active_command->corridor_side=1;
  inputs.opponents[0].x_m=-5.;inputs.leader_vehicle_id="replacement";
  EXPECT_TRUE(node.overtakeTargetValid(inputs));
  WorkBatch batch;batch.maneuver=DrivingMode::OVERTAKE;batch.maneuver_target_id="fast";
  batch.candidates.push_back(*work);
  EXPECT_TRUE(node.maneuverCurrent(batch,inputs));
  inputs.opponents.clear();EXPECT_FALSE(node.maneuverCurrent(batch,inputs));
}

TEST_F(NodeFallbackTest, SteeringUsesLastArrivalAtOrBeforePoseStampAndExactPendingTargets) {
  nav_msgs::msg::Odometry odometry; odometry.header.stamp.sec=100;
  node.odometry_=odometry;
  node.config_.steering_control_delay_sec=.07;node.config_.dt_sec=.05;
  node.steering_command_history_={{99.90,.4,10},{99.95,.6,11}};
  autoware_auto_vehicle_msgs::msg::SteeringReport steering;
  steering.stamp.sec=100;steering.steering_tire_angle=.2F;
  node.receiveSteeringReport(steering);
  steering.steering_tire_angle=.3F;node.receiveSteeringReport(steering);
  const auto expected_sequence=node.input_event_sequence_;
  steering.stamp.nanosec=100000000;steering.steering_tire_angle=.8F;
  node.receiveSteeringReport(steering);
  const auto inputs=node.captureBrainInputs();
  EXPECT_TRUE(inputs.steering_fresh);
  EXPECT_DOUBLE_EQ(inputs.steering_stamp_sec,100.);
  EXPECT_EQ(inputs.steering_sequence,expected_sequence);
  EXPECT_NEAR(inputs.steering_rad,.3,1e-7);
  ASSERT_TRUE(inputs.pending_steering_targets.valid);
  ASSERT_EQ(inputs.pending_steering_targets.count,2U);
  EXPECT_DOUBLE_EQ(inputs.pending_steering_targets.targets_rad[0],.4);
  EXPECT_DOUBLE_EQ(inputs.pending_steering_targets.targets_rad[1],.6);
  EXPECT_EQ(inputs.pending_steering_targets.source_sequences[0],10U);
  EXPECT_EQ(inputs.pending_steering_targets.source_sequences[1],11U);
  EXPECT_DOUBLE_EQ(inputs.pending_steering_targets.source_stamps_sec[0],99.90);
  EXPECT_DOUBLE_EQ(inputs.pending_steering_targets.source_stamps_sec[1],99.95);
}
TEST_F(NodeFallbackTest, LateSteeringDoesNotDiscardMoreRecentEligibleMeasurement) {
  nav_msgs::msg::Odometry odometry;odometry.header.stamp.sec=100;node.odometry_=odometry;
  autoware_auto_vehicle_msgs::msg::SteeringReport steering;
  steering.stamp.sec=99;steering.stamp.nanosec=900000000;steering.steering_tire_angle=.2F;
  node.receiveSteeringReport(steering);
  const auto expected_sequence=node.input_event_sequence_;
  steering.stamp.sec=100;steering.stamp.nanosec=100000000;steering.steering_tire_angle=.8F;
  node.receiveSteeringReport(steering);
  steering.stamp.sec=99;steering.stamp.nanosec=800000000;steering.steering_tire_angle=.1F;
  node.receiveSteeringReport(steering);
  const auto inputs=node.captureBrainInputs();
  EXPECT_TRUE(inputs.steering_fresh);EXPECT_EQ(inputs.steering_sequence,expected_sequence);
  EXPECT_NEAR(inputs.steering_stamp_sec,99.9,1e-9);EXPECT_NEAR(inputs.steering_rad,.2,1e-7);
}
TEST_F(NodeFallbackTest, FutureOnlySteeringIsNotAnObservedInitialSteering) {
  nav_msgs::msg::Odometry odometry;odometry.header.stamp.sec=100;node.odometry_=odometry;
  autoware_auto_vehicle_msgs::msg::SteeringReport steering;
  steering.stamp.sec=101;steering.steering_tire_angle=.8F;
  node.receiveSteeringReport(steering);
  const auto inputs=node.captureBrainInputs();
  EXPECT_FALSE(inputs.steering_fresh);EXPECT_EQ(inputs.steering_sequence,0U);
}
TEST_F(NodeFallbackTest, CollectionFitOwnsTheSnapshotEvenWhenOnlinePredictionIsEnabled) {
  node.collection_motion_enabled_=true;
  node.online_motion_prediction_enabled_=true;
  node.base_reference_=std::make_shared<Trajectory>(desired().trajectory);
  const double now=node.now().seconds();
  const rclcpp::Time source_epoch(static_cast<int64_t>((now-.1)*1e9));
  v2x_msgs::msg::V2XVehiclePositionArray message; message.vehicles.resize(1);
  auto &v=message.vehicles.front(); v.vehicle_id="lidar_test";
  for (int i=0;i<=12;++i) {
    v.header.stamp=source_epoch+rclcpp::Duration::from_seconds(-.6+i*.05);
    v.position.x=20.+(i%2 ? .05 : -.05);
    node.receiveVehiclePositions(message);
  }
  nav_msgs::msg::Odometry odom;
  odom.header.stamp=rclcpp::Time(static_cast<int64_t>(now*1e9));
  odom.pose.pose.orientation.w=1.; node.odometry_=odom;
  const auto inputs=node.captureBrainInputs(); ASSERT_EQ(inputs.opponents.size(),1U);
  const auto &opponent=inputs.opponents.front();
  EXPECT_FALSE(opponent.prediction); EXPECT_DOUBLE_EQ(opponent.vx_mps,0.);
  EXPECT_NEAR(opponent.x_m,20.,.051);
  EXPECT_FALSE(node.observed_vehicles_.at("lidar_test").online_motion);
}

TEST_F(NodeFallbackTest, CollectionContinuationRequiresAdoptedTargetIdentity) {
  BrainInputs inputs;
  inputs.leading=FollowVehicle{};
  inputs.leading->vehicle.id="lidar_a";
  inputs.active_target_id="lidar_a";
  inputs.driving_fsm.acceptLateral();
  EXPECT_FALSE(node.continueCollectionAvoidance(inputs));
  node.collection_avoidance_continuation_=true;
  EXPECT_TRUE(node.continueCollectionAvoidance(inputs));
  inputs.active_target_id="lidar_b";
  EXPECT_FALSE(node.continueCollectionAvoidance(inputs));
  inputs.active_target_id="lidar_a"; inputs.leading.reset();
  EXPECT_FALSE(node.continueCollectionAvoidance(inputs));
}

TEST_F(NodeFallbackTest, DisabledPriorPredictionDoesNotCollectHistory) {
  EXPECT_FALSE(node.prior_lap_prediction_enabled_);
  node.base_reference_=std::make_shared<Trajectory>(desired().trajectory);
  v2x_msgs::msg::V2XVehiclePositionArray message;message.vehicles.resize(1);
  auto &v=message.vehicles.front();v.vehicle_id="d2";v.header.stamp.sec=100;v.position.x=10.;
  node.receiveVehiclePositions(message);
  v.header.stamp.nanosec=100000000;v.position.x=10.5;node.receiveVehiclePositions(message);
  EXPECT_FALSE(node.observed_vehicles_.at("d2").history);
  EXPECT_FALSE(node.observed_vehicles_.at("d2").prediction);
  EXPECT_GT(node.observed_vehicles_.at("d2").vx_mps,0.);
}

TEST_F(NodeFallbackTest, LapReplayFollowsOnlyCurrentLeaderAndKeepsEveryObstacle) {
  stopBrainWorker();
  node.leader_lap_prediction_enabled_=true;
  node.recent_motion_prediction_enabled_=true;
  node.base_reference_=std::make_shared<Trajectory>();
  for (int i=0;i<=1000;++i) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint p;
    const double a=2.*M_PI*i/1000.;
    p.pose.position.x=20.*std::cos(a);p.pose.position.y=20.*std::sin(a);
    p.pose.orientation=quaternionFromYaw(a+M_PI/2.);
    p.longitudinal_velocity_mps=9.;
    node.base_reference_->points.push_back(p);
  }
  const double now=node.now().seconds();
  auto history=std::make_shared<std::vector<opponent_prediction::PositionObservation>>();
  for (int i=0;i<=2100;++i) {
    const double t=i*.02,a=2.*M_PI*t/20.;
    history->push_back({now-42.+t,20.*std::cos(a),20.*std::sin(a),20.*a});
  }
  for (const std::string id:{"d2","d3"}) {
    ObservedVehicle v;v.id=id;v.history=history;
    v.stamp_sec=v.receive_sec=now;
    v.x_m=history->back().x;v.y_m=history->back().y;
    node.observed_vehicles_[id]=v;
  }
  nav_msgs::msg::Odometry odom;odom.header.stamp=rclcpp::Time(static_cast<int64_t>(now*1e9));
  node.odometry_=odom;node.leader_receive_sec_=now;
  for (const std::string leader:{"d3","d2","__ego",""}) {
    node.leader_vehicle_id_=leader;
    const auto inputs=node.captureBrainInputs();
    ASSERT_EQ(inputs.opponents.size(),2U);
    for (const auto &v:inputs.opponents) {
      EXPECT_EQ(static_cast<bool>(v.leader_lap_prediction),v.id==leader);
      ASSERT_TRUE(v.prediction);
    }
    mppi::PlanRequest request;
    const auto ego=node.projectOnTrajectory(*node.base_reference_,20.,0.);
    ASSERT_TRUE(node.populateDynamicObstacles(request,inputs,ego,
        node.trajectoryArcLength(*node.base_reference_),true));
    EXPECT_EQ(request.dynamic_obstacle_count,2U);
    EXPECT_EQ(request.overtake_target_index,std::numeric_limits<std::size_t>::max());
  }
  node.leader_vehicle_id_="d3";node.leader_receive_sec_=now-10.;
  for (const auto &v:node.captureBrainInputs().opponents) EXPECT_FALSE(v.leader_lap_prediction);
}
TEST_F(NodeFallbackTest, OpponentHistoryRetainsLatestPositionForTheSameSourceStamp) {
  node.prior_lap_prediction_enabled_=true;
  node.base_reference_=std::make_shared<Trajectory>(desired().trajectory);
  v2x_msgs::msg::V2XVehiclePositionArray message;
  message.vehicles.resize(1);
  auto &v=message.vehicles.front();v.vehicle_id="d2";v.header.stamp.sec=100;
  v.position.x=10.;node.receiveVehiclePositions(message);
  v.position.x=11.;node.receiveVehiclePositions(message);
  const auto &observed=node.observed_vehicles_.at("d2");
  ASSERT_TRUE(observed.history);ASSERT_EQ(observed.history->size(),1U);
  EXPECT_DOUBLE_EQ(observed.history->back().x,observed.x_m);
  EXPECT_DOUBLE_EQ(observed.history->back().x,11.);
  v.header.stamp.sec=99;v.position.x=12.;node.receiveVehiclePositions(message);
  EXPECT_DOUBLE_EQ(node.observed_vehicles_.at("d2").history->back().x,11.);
}
TEST_F(NodeFallbackTest, RecentMotionRetainsTimeBoundedHistoryOnAnOpenReference) {
  node.recent_motion_prediction_enabled_=true;
  node.base_reference_=std::make_shared<Trajectory>(desired().trajectory);
  const double now=node.now().seconds();
  v2x_msgs::msg::V2XVehiclePositionArray message;message.vehicles.resize(1);
  auto &v=message.vehicles.front();v.vehicle_id="d2";
  for(int i=0;i<=40;++i) {
    const double dt=-2.+i*.05;
    v.header.stamp=rclcpp::Time(static_cast<int64_t>((now+dt)*1e9));
    v.position.x=10.+dt;node.receiveVehiclePositions(message);
  }
  const auto &history=node.observed_vehicles_.at("d2").history;
  ASSERT_TRUE(history);EXPECT_LE(history->size(),14U);
  nav_msgs::msg::Odometry odom;odom.header.stamp=v.header.stamp;odom.pose.pose.orientation.w=1.;
  node.odometry_=odom;
  const auto inputs=node.captureBrainInputs();ASSERT_EQ(inputs.opponents.size(),1U);
  const auto &opponent=inputs.opponents.front();ASSERT_TRUE(opponent.prediction);
  EXPECT_NEAR(opponent.x_m,10.,1e-5);EXPECT_NEAR(opponent.vx_mps,1.,1e-5);
  mppi::PlanRequest request;
  const auto ego=node.projectOnTrajectory(*node.base_reference_,0.,0.);
  ASSERT_TRUE(node.populateDynamicObstacles(request,inputs,ego,40.,false));
  EXPECT_EQ(request.dynamic_obstacles[0].prediction,opponent.prediction);
}
TEST_F(NodeFallbackTest, OnlineMotionLearnsPerOpponentAndFreezesThePlanningSnapshot) {
  node.online_motion_prediction_enabled_=true;
  node.base_reference_=std::make_shared<Trajectory>(desired().trajectory);
  const double now=node.now().seconds();
  v2x_msgs::msg::V2XVehiclePositionArray message;message.vehicles.resize(2);
  message.vehicles[0].vehicle_id="d2";message.vehicles[1].vehicle_id="d3";
  for(int i=0;i<=60;++i) {
    const double t=i*.05;
    for(auto &v:message.vehicles)v.header.stamp=rclcpp::Time(static_cast<int64_t>((now-3.+t)*1e9));
    message.vehicles[0].position.x=10.+2.*t;
    message.vehicles[1].position.x=10.+10.*std::sin(.3*t);
    message.vehicles[1].position.y=10.*(1.-std::cos(.3*t));
    node.receiveVehiclePositions(message);
  }
  const auto learner=node.observed_vehicles_.at("d2").online_motion;
  ASSERT_TRUE(learner);EXPECT_GT(learner->summary().samples[2],0U);
  EXPECT_NE(learner,node.observed_vehicles_.at("d3").online_motion);
  nav_msgs::msg::Odometry odom;odom.header.stamp=message.vehicles[0].header.stamp;
  odom.pose.pose.orientation.w=1.;node.odometry_=odom;
  const auto inputs=node.captureBrainInputs();ASSERT_EQ(inputs.opponents.size(),2U);
  ASSERT_TRUE(inputs.opponents[0].prediction);
  mppi::PlanRequest request;
  const auto ego=node.projectOnTrajectory(*node.base_reference_,0.,0.);
  ASSERT_TRUE(node.populateDynamicObstacles(request,inputs,ego,40.,false));
  EXPECT_EQ(request.dynamic_obstacles[0].prediction,inputs.opponents[0].prediction);
  EXPECT_EQ(request.dynamic_obstacles[1].prediction,inputs.opponents[1].prediction);
  const auto samples=learner->summary().samples;
  message.vehicles[0].header.stamp=rclcpp::Time(static_cast<int64_t>((now+.1)*1e9));
  message.vehicles[0].position.x=16.2;node.receiveVehiclePositions(message);
  EXPECT_EQ(learner->summary().samples,samples);
  EXPECT_NE(learner,node.observed_vehicles_.at("d2").online_motion);
}
TEST_F(NodeFallbackTest, OnlineBrakingUsesTheSameAlignedStateInPredictionAndObstacle) {
  node.online_motion_prediction_enabled_=true;
  node.base_reference_=std::make_shared<Trajectory>(desired().trajectory);
  const double now=node.now().seconds();
  v2x_msgs::msg::V2XVehiclePositionArray message;message.vehicles.resize(1);
  auto &v=message.vehicles.front();v.vehicle_id="d2";
  const rclcpp::Time source_epoch(static_cast<int64_t>((now-.2)*1e9));
  for(int i=0;i<=12;++i) {
    const double t=-.6+i*.05;
    v.header.stamp=source_epoch+rclcpp::Duration::from_seconds(t);
    // Match the synthetic motion to the encoded stamp at Unix-time magnitudes.
    const double encoded_t=rclcpp::Time(v.header.stamp).seconds()-source_epoch.seconds();
    v.position.x=20.+6.*encoded_t-2.*encoded_t*encoded_t;node.receiveVehiclePositions(message);
  }
  nav_msgs::msg::Odometry odom;odom.header.stamp=rclcpp::Time(static_cast<int64_t>(now*1e9));
  odom.pose.pose.orientation.w=1.;node.odometry_=odom;
  const auto inputs=node.captureBrainInputs();ASSERT_EQ(inputs.opponents.size(),1U);
  const auto &opponent=inputs.opponents.front();ASSERT_TRUE(opponent.prediction);
  EXPECT_NEAR(opponent.x_m,21.12,1e-5);EXPECT_NEAR(opponent.vx_mps,5.2,1e-5);
  EXPECT_NEAR(opponent.prediction->at(.8).global_s,24.32,1e-5);
  mppi::PlanRequest request;
  const auto ego=node.projectOnTrajectory(*node.base_reference_,0.,0.);
  ASSERT_TRUE(node.populateDynamicObstacles(request,inputs,ego,40.,false));
  ASSERT_EQ(request.dynamic_obstacle_count,1U);
  const auto &obstacle=request.dynamic_obstacles[0];
  EXPECT_EQ(obstacle.prediction,opponent.prediction);
  EXPECT_NEAR(obstacle.global_reference_s_m,opponent.prediction->points.front().global_s,1e-5);
  EXPECT_NEAR(obstacle.longitudinal_speed_mps,opponent.vx_mps,1e-5);
}
TEST_F(NodeFallbackTest, OpponentMarkersUseTheExistingAlignedForecastAndTimeSamples) {
  using Marker = visualization_msgs::msg::Marker;
  BrainInputs inputs;
  inputs.base_reference = std::make_shared<Trajectory>(desired().trajectory);
  inputs.base_reference->header.frame_id = "test_map";
  inputs.odometry.emplace(); inputs.odometry->header.stamp.sec = 100;
  ObservedVehicle opponent;
  opponent.id = "d2"; opponent.x_m = -100.; opponent.vx_mps = 99.;
  auto prediction = std::make_shared<opponent_prediction::Prediction>();
  prediction->source_stamp = 99.8;
  prediction->points = {{0.,10.,1.,0.}, {1.,14.,2.,.1}, {2.,16.,3.,.2}};
  opponent.prediction = prediction;
  inputs.opponents.push_back(opponent);
  const auto markers = node.makeOpponentPredictionMarkers(inputs);
  ASSERT_FALSE(markers.markers.empty());
  EXPECT_EQ(markers.markers.front().action, Marker::DELETEALL);
  bool saw_path = false, saw_ticks = false, saw_one_second = false;
  for (const auto &m : markers.markers) {
    if (m.action != Marker::ADD) continue;
    EXPECT_EQ(m.header.frame_id, "test_map");
    EXPECT_EQ(m.header.stamp.sec, 100);
    EXPECT_GT(rclcpp::Duration(m.lifetime).seconds(), 0.);
    if (m.type == Marker::LINE_STRIP) {
      saw_path = true;
      EXPECT_DOUBLE_EQ(m.points.front().x, 10.);
      EXPECT_DOUBLE_EQ(m.points.front().y, 1.);
      EXPECT_DOUBLE_EQ(m.points.back().x, 16.);
      EXPECT_DOUBLE_EQ(m.points.back().y, 3.);
    } else if (m.type == Marker::SPHERE_LIST) {
      saw_ticks = true;
      ASSERT_EQ(m.points.size(), 2U);
      EXPECT_DOUBLE_EQ(m.points[0].x, 14.);
      EXPECT_DOUBLE_EQ(m.points[0].y, 2.);
      EXPECT_DOUBLE_EQ(m.points[1].x, 16.);
    } else if (m.text == "+1s") {
      saw_one_second = true;
      EXPECT_DOUBLE_EQ(m.pose.position.x, 14.);
      EXPECT_DOUBLE_EQ(m.pose.position.y, 2.);
    }
  }
  EXPECT_TRUE(saw_path); EXPECT_TRUE(saw_ticks); EXPECT_TRUE(saw_one_second);
  EXPECT_DOUBLE_EQ(prediction->source_stamp, 99.8);
  EXPECT_EQ(inputs.opponents.front().prediction, prediction);
}
TEST_F(NodeFallbackTest, OpponentMarkersClearMissingPredictionsAndAvoidStackedStoppedLabels) {
  using Marker = visualization_msgs::msg::Marker;
  BrainInputs inputs;
  inputs.base_reference = std::make_shared<Trajectory>(desired().trajectory);
  ObservedVehicle opponent; opponent.id = "d2";
  auto prediction = std::make_shared<opponent_prediction::Prediction>();
  prediction->points = {{0.,10.,1.,0.}, {3.,10.,1.,0.}};
  opponent.prediction = prediction; inputs.opponents.push_back(opponent);
  const auto stopped = node.makeOpponentPredictionMarkers(inputs);
  EXPECT_EQ(std::count_if(stopped.markers.begin(), stopped.markers.end(),
      [](const auto &m) { return m.action == Marker::ADD && m.type == Marker::TEXT_VIEW_FACING; }), 1);
  inputs.opponents.front().prediction.reset();
  const auto missing = node.makeOpponentPredictionMarkers(inputs);
  ASSERT_EQ(missing.markers.size(), 1U);
  EXPECT_EQ(missing.markers.front().action, Marker::DELETEALL);
  inputs.base_reference.reset();
  const auto no_reference = node.makeOpponentPredictionMarkers(inputs);
  ASSERT_EQ(no_reference.markers.size(), 1U);
  EXPECT_EQ(no_reference.markers.front().action, Marker::DELETEALL);
}
TEST_F(NodeFallbackTest, ParkedOpponentHistoryHasBoundedStorageAndKeepsRecentFit) {
  node.prior_lap_prediction_enabled_=true;
  node.base_reference_=std::make_shared<Trajectory>(desired().trajectory);
  v2x_msgs::msg::V2XVehiclePositionArray message;
  message.vehicles.resize(1);
  auto &v=message.vehicles.front();v.vehicle_id="d2";v.position.x=10.;
  for(int i=0;i<8200;++i) {
    v.header.stamp=rclcpp::Time(100000000000LL+50000000LL*i);
    node.receiveVehiclePositions(message);
  }
  const auto &history=*node.observed_vehicles_.at("d2").history;
  EXPECT_LE(history.size(),8192U);
  std::array<double,4> state;
  ASSERT_TRUE(opponent_prediction::fitRecent(history,state));
  EXPECT_NEAR(state[0],10.,1e-9);EXPECT_NEAR(state[2],0.,1e-9);
}
TEST_F(NodeFallbackTest, WallValidatorsRetainTheirMapAndIndexSnapshot) {
  auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.resolution = .1F; map->info.width = 1000; map->info.height = 100;
  map->info.origin.position.x = -10.; map->info.origin.position.y = -5.;
  map->info.origin.orientation.w = 1.; map->data.assign(100000, 0);
  BrainInputs inputs;
  inputs.base_reference = std::make_shared<Trajectory>(desired().trajectory);
  inputs.wall_map = map;
  inputs.wall_index = std::make_shared<OccupancyGridWallIndex>(map);
  nav_msgs::msg::Odometry ego; ego.pose.pose.orientation.w = 1.;
  const auto old_path = node.makeBrainPathConstraintValidator(inputs, ego, 0.);
  const auto old_rollout = node.makeBrainRolloutConstraintValidator(inputs);
  auto updated = std::make_shared<nav_msgs::msg::OccupancyGrid>(*map);
  updated->data[50U * updated->info.width + 155U] = 100;
  inputs.wall_map = updated;
  inputs.wall_index = std::make_shared<OccupancyGridWallIndex>(updated);
  const auto new_path = node.makeBrainPathConstraintValidator(inputs, ego, 0.);
  const auto new_rollout = node.makeBrainRolloutConstraintValidator(inputs);
  const auto path = node.temporaryReferenceFromCommand(desired());
  EXPECT_EQ(old_path(path), mppi::RejectReason::NONE);
  EXPECT_EQ(new_path(path), mppi::RejectReason::WALL);
  mppi::RolloutState a, b;
  a.x_m = 5.; b.x_m = 6.; b.time_sec = .1;
  EXPECT_EQ(old_rollout(a, b), mppi::RejectReason::NONE);
  EXPECT_EQ(new_rollout(a, b), mppi::RejectReason::WALL);
  EXPECT_EQ(old_path(path), mppi::RejectReason::NONE);
}

TEST_F(NodeFallbackTest, HeartbeatCannotReplaceNewGeometryWithOldValidatedHold) {
  BrainInputs snapshot;
  snapshot.active_command = desired();
  node.active_brain_command_ = desired();
  ++node.active_brain_command_->generation;
  EXPECT_FALSE(node.publishHeartbeatCommand(snapshot, desired()));
}
TEST_F(NodeFallbackTest, RecoveryInvalidatesOldValidationWithoutChangingShapeOwner) {
  BrainInputs snapshot;
  snapshot.active_command = desired();
  node.active_brain_command_ = desired();
  ExecutionRevisionCheck pending{node.execution_revision_};
  auto recovery=desired();
  recovery.reason="mppi_brain:infeasible_braking_fallback";
  recovery.trajectory.points.front().longitudinal_velocity_mps=0.;
  ASSERT_TRUE(node.publishHeartbeatCommand(snapshot,recovery));
  EXPECT_EQ(node.active_brain_command_->generation,42U);
  EXPECT_FALSE(pending.canCommit(node.execution_revision_));
  EXPECT_FALSE(node.publishHeartbeatCommand(snapshot,desired()));
  snapshot.execution_revision=node.execution_revision_;
  pending.recordValidation(snapshot.execution_revision,true);
  EXPECT_TRUE(pending.canCommit(node.execution_revision_));
  ASSERT_TRUE(node.publishHeartbeatCommand(snapshot,desired()));
  // Leaving recovery is a real execution change, not a resend.
  EXPECT_FALSE(pending.canCommit(node.execution_revision_));
  snapshot.execution_revision=node.execution_revision_;
  pending.recordValidation(snapshot.execution_revision,true);
  ASSERT_TRUE(node.publishHeartbeatCommand(snapshot,desired()));
  EXPECT_TRUE(pending.canCommit(node.execution_revision_));
  ASSERT_TRUE(node.publishHeartbeatCommand(snapshot,recovery));
  EXPECT_FALSE(pending.canCommit(node.execution_revision_));
}
TEST_F(NodeFallbackTest, IdenticalRecoveryResendKeepsRevisionButChangedTailInvalidates) {
  BrainInputs snapshot;
  snapshot.active_command=desired(); node.active_brain_command_=desired();
  auto recovery=desired(); recovery.reason="mppi_brain:hold_speed_recovery";
  ASSERT_TRUE(node.publishHeartbeatCommand(snapshot,recovery));
  snapshot.execution_revision=node.execution_revision_;
  const auto revision=node.execution_revision_;
  ++recovery.generation; ++recovery.header.stamp.sec;
  ++recovery.trajectory.header.stamp.sec; recovery.valid_until_sec+=1.;
  ASSERT_TRUE(node.publishHeartbeatCommand(snapshot,recovery));
  EXPECT_EQ(node.execution_revision_,revision);
  recovery.trajectory.points.back().longitudinal_velocity_mps-=1.;
  ASSERT_TRUE(node.publishHeartbeatCommand(snapshot,recovery));
  EXPECT_NE(node.execution_revision_,revision);
  EXPECT_FALSE(node.publishHeartbeatCommand(snapshot,desired()));
}
TEST_F(NodeFallbackTest, SameSpeedDifferentGeometryIsNotRecoveryResend) {
  BrainInputs snapshot; auto recovery=desired();
  recovery.reason="mppi_brain:validated_braking_fallback";
  ASSERT_TRUE(node.publishHeartbeatCommand(snapshot,recovery));
  snapshot.execution_revision=node.execution_revision_;
  recovery.trajectory.points.back().pose.position.y+=0.01;
  ASSERT_TRUE(node.publishHeartbeatCommand(snapshot,recovery));
  EXPECT_NE(node.execution_revision_,snapshot.execution_revision);
}
// Characterization tests: these reproduce the current invalidation mechanisms,
// not the desired behavior of a future scheduling fix.
TEST_F(NodeFallbackTest, ValidationThenHeartbeatThenCommitLosesValidatedRevision) {
  BrainInputs snapshot;
  snapshot.active_command=desired(); node.active_brain_command_=desired();
  auto recovery=desired(); recovery.reason="mppi_brain:hold_speed_recovery";
  ASSERT_TRUE(node.publishHeartbeatCommand(snapshot,recovery));
  snapshot.execution_revision=node.execution_revision_;
  ExecutionRevisionCheck worker{snapshot.execution_revision};
  worker.recordValidation(snapshot.execution_revision,true);
  ASSERT_TRUE(worker.canCommit(node.execution_revision_));
  // Fix the observed interleaving without scheduler timing or sleeps:
  // validation completes, heartbeat holds decision, then worker acquires it.
  {
    recovery.trajectory.points.back().longitudinal_velocity_mps-=1.;
    ASSERT_TRUE(node.publishHeartbeatCommand(snapshot,recovery));
  }
  {
    std::lock_guard<std::mutex> decision(node.decision_mutex_);
    std::lock_guard<std::mutex> authority(node.authority_mutex_);
    EXPECT_EQ(node.active_brain_command_->generation,snapshot.active_command->generation);
    EXPECT_FALSE(worker.canCommit(node.execution_revision_));
    worker.recordValidation(node.execution_revision_,true);
    EXPECT_TRUE(worker.canCommit(node.execution_revision_));
    worker.recordValidation(node.execution_revision_,false);
    EXPECT_FALSE(worker.canCommit(node.execution_revision_));
  }
}
TEST_F(NodeFallbackTest, SlowHeartbeatPreparationAllowsWorkerCommitAndRejectsOldResult) {
  node.active_brain_command_=desired();
  const auto snapshot=node.captureBrainInputs();
  std::promise<void> entered, resume;
  auto resume_future=resume.get_future().share();
  bool first=true, committed=false;
  auto heartbeat=std::async(std::launch::async,[&] {
    const auto recovered=node.recoverBrainCommandSpeed(desired(),[&](const auto &) {
      if(first) { first=false; entered.set_value(); resume_future.wait(); }
      return mppi::RejectReason::NONE;
    });
    return recovered && node.publishHeartbeatCommand(snapshot,*recovered,[&] {committed=true;});
  });
  entered.get_future().wait();
  {
    std::lock_guard<std::mutex> decision(node.decision_mutex_);
    std::lock_guard<std::mutex> authority(node.authority_mutex_);
    std::lock_guard<std::mutex> input(node.input_mutex_);
    ++node.active_brain_speed_generation_;
  }
  resume.set_value();
  EXPECT_FALSE(heartbeat.get());
  EXPECT_FALSE(committed);
}
TEST_F(NodeFallbackTest, RecoveryEpochRejectsPreparedHeartbeatAfterRecoveryEnds) {
  const auto snapshot=node.captureBrainInputs();
  ++node.recovery_epoch_;
  EXPECT_FALSE(node.publishHeartbeatCommand(snapshot,desired()));
  EXPECT_FALSE(node.setSemanticAuthority(123U,snapshot));
}
TEST_F(NodeFallbackTest, InputArrivalDoesNotInvalidatePreparedHeartbeat) {
  const auto snapshot=node.captureBrainInputs();
  ++node.input_event_sequence_;
  EXPECT_TRUE(node.publishHeartbeatCommand(snapshot,desired()));
  EXPECT_TRUE(node.setSemanticAuthority(123U,snapshot));
}
TEST_F(NodeFallbackTest, ProvenRecoveryProjectionKeepsPlanRevision) {
  BrainInputs snapshot;
  snapshot.active_command=desired(); node.active_brain_command_=desired();
  nav_msgs::msg::Odometry ego;
  ego.pose.pose.orientation.w=1.; ego.twist.twist.linear.x=2.;
  ego.pose.pose.position.x=1.;
  const auto heartbeat=desired();
  snapshot.odometry=ego;
  snapshot.base_reference=std::make_shared<Trajectory>(heartbeat.trajectory);
  auto first=node.refreshedBrainHold(*snapshot.active_command,heartbeat,ego,heartbeat.trajectory);
  ASSERT_GT(first.trajectory.points.size(),2U);
  first.reason="mppi_brain:hold_speed_recovery";
  for(auto &point:first.trajectory.points) point.longitudinal_velocity_mps*=.5f;
  ASSERT_TRUE(node.publishHeartbeatCommand(snapshot,first));
  snapshot.execution_revision=node.execution_revision_;
  ego.pose.pose.position.x=1.1;
  snapshot.odometry=ego;
  auto next=node.refreshedBrainHold(*snapshot.active_command,heartbeat,ego,heartbeat.trajectory);
  ASSERT_GT(next.trajectory.points.size(),2U);
  next.reason=first.reason;
  for(auto &point:next.trajectory.points) point.longitudinal_velocity_mps*=.5f;
  EXPECT_EQ(next.geometry_revision,first.geometry_revision);
  EXPECT_NE(next.trajectory,first.trajectory);
  ASSERT_TRUE(node.publishHeartbeatCommand(snapshot,next));
  EXPECT_EQ(node.execution_revision_,snapshot.execution_revision);
  EXPECT_EQ(node.active_brain_command_,snapshot.active_command);
  next.trajectory.points.back().longitudinal_velocity_mps-=.1f;
  ASSERT_TRUE(node.publishHeartbeatCommand(snapshot,next));
  EXPECT_NE(node.execution_revision_,snapshot.execution_revision);
}
TEST_F(NodeFallbackTest, GeometryRevisionFieldAloneKeepsResendRevision) {
  BrainInputs snapshot;
  auto first=desired(); first.reason="mppi_brain:feasible_speed_fallback";
  ASSERT_TRUE(node.publishHeartbeatCommand(snapshot,first));
  snapshot.execution_revision=node.execution_revision_;
  auto next=first; ++next.geometry_revision;
  ASSERT_EQ(next.trajectory,first.trajectory);
  ASSERT_TRUE(node.publishHeartbeatCommand(snapshot,next));
  EXPECT_EQ(node.execution_revision_,snapshot.execution_revision);
}
TEST_F(NodeFallbackTest, SelectedExecutionDoesNotSpendEvaluationsOnUnusedFallback) {
  WorkBatch batch; batch.brain_owned=true; batch.fallback=desired();
  int calls=0;
  batch.fallback_path_constraint_validator=[&](const mppi::TemporaryReference &) {
    ++calls; return mppi::RejectReason::NONE;
  };
  for (const auto selected : {ExecutionSelection{true,0}, ExecutionSelection{false,0},
                              ExecutionSelection{false,4}}) {
    node.makeBatchFallback(batch,selected);
  }
  EXPECT_EQ(calls,0);
}
TEST_F(NodeFallbackTest, NoSelectionPreservesFallbackRecoveryAndEvaluationOrder) {
  WorkBatch batch; batch.brain_owned=true; batch.fallback=desired();
  std::vector<double> calls;
  batch.fallback_path_constraint_validator=[&](const mppi::TemporaryReference &r) {
    calls.push_back(r.points[0].speed_mps);
    return r.points[0].speed_mps<=6. ? mppi::RejectReason::NONE : mppi::RejectReason::COLLISION;
  };
  const auto expected=node.physicallyBoundedFallback(batch.fallback,batch.fallback_path_constraint_validator);
  const auto expected_calls=calls; calls.clear();
  const auto actual=node.makeBatchFallback(batch,std::nullopt);
  EXPECT_EQ(calls,expected_calls);
  EXPECT_EQ(actual.reason,expected.reason);
  ASSERT_EQ(actual.trajectory.points.size(),expected.trajectory.points.size());
  for (std::size_t i=0;i<actual.trajectory.points.size();++i)
    EXPECT_EQ(actual.trajectory.points[i].longitudinal_velocity_mps,
              expected.trajectory.points[i].longitudinal_velocity_mps);
}
TEST_F(NodeFallbackTest, WallLimitUsesFreeSpaceBetweenCoarseSamplesOnBothSides) {
  nav_msgs::msg::OccupancyGrid map;
  map.info.resolution=.01; map.info.width=map.info.height=600;
  map.info.origin.position.x=map.info.origin.position.y=-3.;
  map.info.origin.orientation.w=1.; map.data.assign(600*600,0);
  for (unsigned y=0;y<600;++y) for (unsigned x=0;x<600;++x)
    if (y>=438 || y<162) map.data[y*600+x]=100;
  mppi::BaseReferencePoint base;
  const auto indexed_map=std::make_shared<const nav_msgs::msg::OccupancyGrid>(map);
  const OccupancyGridWallIndex index(indexed_map);
  for (int side : {-1,1}) {
    const double limit=node.lateralLimit(map,base,side);
    EXPECT_DOUBLE_EQ(node.lateralLimit(*indexed_map,base,side,&index),limit);
    EXPECT_GT(limit,.72);
    EXPECT_LT(limit,.731);
    EXPECT_TRUE(node.footprintFree(map,0.,side*limit,0.));
    EXPECT_FALSE(node.footprintFree(map,0.,side*(limit+.002),0.));
  }
  for(double yaw:{-.4,.2,.9}) for(int side:{-1,1}) {
    base.yaw_rad=yaw;
    EXPECT_DOUBLE_EQ(node.lateralLimit(*indexed_map,base,side,&index),node.lateralLimit(map,base,side));
  }
}
TEST_F(NodeFallbackTest, HeartbeatCannotUndoSpeedRetimeOnIdenticalGeometry) {
  BrainInputs snapshot;
  snapshot.active_command = desired();
  snapshot.speed_generation = 7;
  node.active_brain_command_ = desired();
  node.active_brain_speed_generation_ = 8;
  EXPECT_FALSE(node.publishHeartbeatCommand(snapshot, desired()));
}
TEST_F(NodeFallbackTest, PlanningFallbackCannotReplaceFirstAcceptedOvertake) {
  BrainInputs snapshot;
  node.active_brain_command_ = desired();
  EXPECT_FALSE(node.publishHeartbeatCommand(snapshot, desired()));
}
TEST_F(NodeFallbackTest, CurrentHeartbeatStillPublishesIncludingNoActivePlan) {
  BrainInputs snapshot;
  EXPECT_TRUE(node.publishHeartbeatCommand(snapshot, desired()));
  snapshot.active_command = node.active_brain_command_ = desired();
  snapshot.speed_generation = node.active_brain_speed_generation_ = 7;
  EXPECT_TRUE(node.publishHeartbeatCommand(snapshot, desired()));
  node.driving_fsm_.acceptLateral();
  EXPECT_FALSE(node.publishHeartbeatCommand(snapshot, desired()));
}
TEST_F(NodeFallbackTest, StaleHeartbeatCannotClearNewExecutionOwner) {
  BrainInputs snapshot;
  node.active_brain_command_ = desired();
  bool committed = false;
  EXPECT_FALSE(node.publishHeartbeatCommand(snapshot, desired(), [&] {
    committed = true;
    node.clearBrainExecution();
  }));
  EXPECT_FALSE(committed);
  ASSERT_TRUE(node.active_brain_command_);
  EXPECT_EQ(node.active_brain_command_->generation, 42U);
  snapshot.active_command = desired();
  EXPECT_TRUE(node.publishHeartbeatCommand(snapshot, desired(), [&] {
    committed = true;
    node.clearBrainExecution();
  }));
  EXPECT_TRUE(committed);
  EXPECT_FALSE(node.active_brain_command_);
}

TEST_F(NodeFallbackTest, FollowUsesThreeMetresOfBodyFreeSpace) {
  const auto out = followAt(3.);
  ASSERT_FALSE(out.trajectory.points.empty());
  for (const auto &p : out.trajectory.points) EXPECT_FLOAT_EQ(p.longitudinal_velocity_mps, 1.);
  EXPECT_EQ(out.mode, "FREE_RUN");
}
TEST_F(NodeFallbackTest, FollowBodyGapUsesMeasuredEgoYawAndTargetVelocityHeading) {
  const auto base = desired();
  nav_msgs::msg::Odometry ego;
  ego.pose.pose.position.x = 5.;
  ego.pose.pose.orientation = quaternionFromYaw(M_PI / 2.);
  BrainInputs inputs;
  ObservedVehicle lead;
  lead.id = "turning";
  lead.vx_mps = std::cos(.5);
  lead.vy_mps = std::sin(.5);
  lead.x_m = 5. + .65 + 1.10 * std::cos(.5) + .65 * std::sin(.5) + 3.;
  inputs.opponents.push_back(lead);
  const auto target = node.selectFollowVehicle(base.trajectory, base.trajectory, ego, inputs);
  ASSERT_TRUE(target);
  EXPECT_NEAR(target->body_gap_m, 3., 1.e-9);
  EXPECT_NEAR(node.makeBrainFollowCommand(base, target, node.driving_fsm_).trajectory.points.front().longitudinal_velocity_mps,
              lead.vx_mps, 1.e-6);
}
TEST_F(NodeFallbackTest, FollowBodyGapUnwrapsAcrossClosedReferenceStart) {
  Trajectory global;
  for (const auto &xy : std::array<std::array<double, 2>, 5>{{
           {{0., 0.}}, {{20., 0.}}, {{20., 20.}}, {{0., 20.}}, {{0., 0.}}}}) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = xy[0]; point.pose.position.y = xy[1];
    global.points.push_back(point);
  }
  nav_msgs::msg::Odometry ego;
  ego.pose.pose.position.y = 2.;
  ego.pose.pose.orientation = quaternionFromYaw(-M_PI / 2.);
  BrainInputs inputs;
  ObservedVehicle lead;
  lead.id = "next_lap"; lead.x_m = 3.16; lead.vx_mps = 1.;
  inputs.opponents.push_back(lead);
  const auto target = node.selectFollowVehicle(global, global, ego, inputs);
  ASSERT_TRUE(target);
  EXPECT_NEAR(target->relative_s_m, 5.16, 1.e-9);
  EXPECT_NEAR(target->body_gap_m, 3., 1.e-9);
}
TEST_F(NodeFallbackTest, IntermediateSpeedRetainsGeometryModeAndLease) {
  const auto c=desired(); int calls=0;
  const auto out=node.physicallyBoundedFallback(c,[&](const mppi::TemporaryReference &r) {
    ++calls; return r.points[0].speed_mps==6. ? mppi::RejectReason::NONE : mppi::RejectReason::WALL;
  });
  EXPECT_EQ(out.trajectory.points[0].longitudinal_velocity_mps,6.);
  EXPECT_EQ(out.mode,c.mode); EXPECT_EQ(out.generation,c.generation);
  EXPECT_EQ(out.header,c.header); EXPECT_EQ(out.valid_until_sec,c.valid_until_sec);
  ASSERT_EQ(out.trajectory.points.size(),c.trajectory.points.size());
  for (std::size_t i=0; i<c.trajectory.points.size(); ++i)
    EXPECT_EQ(out.trajectory.points[i].pose,c.trajectory.points[i].pose);
  EXPECT_LE(calls,11);
}
TEST_F(NodeFallbackTest, AllRejectedStillReportsInfeasibleBraking) {
  const auto out=node.physicallyBoundedFallback(desired(),[](const mppi::TemporaryReference &) {
    return mppi::RejectReason::WALL;
  });
  EXPECT_EQ(out.reason,"mppi_brain:infeasible_braking_fallback");
  for (const auto &p:out.trajectory.points) EXPECT_EQ(p.longitudinal_velocity_mps,0.);
}
TEST_F(NodeFallbackTest, ValidDesiredIncludingStopIsUnchanged) {
  auto c=desired(); for (auto &p:c.trajectory.points)p.longitudinal_velocity_mps=0.;
  int calls=0;
  const auto out=node.physicallyBoundedFallback(c,[&](const mppi::TemporaryReference &) {
    ++calls; return mppi::RejectReason::NONE;
  });
  EXPECT_EQ(c,out); EXPECT_EQ(calls,1);
}
TEST_F(NodeFallbackTest, ManeuverReturnUsesActualXYTangentNotOnlyPoseYaw) {
  const auto c=desired(); nav_msgs::msg::Odometry odometry;
  odometry.pose.pose.position.y=.8;
  odometry.pose.pose.orientation=quaternionFromYaw(.5);
  odometry.twist.twist.linear.x=5.;
  const auto result=node.makeBrainBaseCommand(c,c.trajectory,odometry);
  ASSERT_TRUE(result);
  const auto &points=result->trajectory.points;
  const auto &a=points[0].pose.position, &b=points[1].pose.position;
  EXPECT_NEAR(std::atan2(b.y-a.y,b.x-a.x),.5,.03);
  EXPECT_EQ(a,odometry.pose.pose.position);
  EXPECT_EQ(points.back().pose,c.trajectory.points.back().pose);
}
TEST_F(NodeFallbackTest, OrdinaryRouteKeepsCurvatureFeedbackInsteadOfResettingAnEntry) {
  const auto c=desired(); nav_msgs::msg::Odometry odometry;
  odometry.pose.pose.position.y=.8;
  odometry.pose.pose.orientation=quaternionFromYaw(.5);
  odometry.twist.twist.linear.x=5.;
  const auto result=node.makeBrainBaseCommand(c,c.trajectory,odometry,0.,false);
  ASSERT_TRUE(result);
  const auto &points=result->trajectory.points;
  const auto &a=points[0].pose.position, &b=points[1].pose.position;
  EXPECT_NEAR(std::atan2(b.y-a.y,b.x-a.x),0.,.01);
}

TEST_F(NodeFallbackTest, FollowMatchesAtFiveMetresAndCruisesOutsideBoundary) {
  for (const auto &example : {std::pair<double,double>{5.0, 2.0}, {4.9, 2.0}, {5.1, 10.0}}) {
    const auto command = followAt(example.first, "lead", 2.0);
    ASSERT_FALSE(command.trajectory.points.empty());
    EXPECT_FLOAT_EQ(command.trajectory.points.front().longitudinal_velocity_mps, example.second);
    EXPECT_EQ(command.mode, "FREE_RUN");
  }
  EXPECT_FLOAT_EQ(followAt(4.0, "stopped", 0.0).trajectory.points.front().longitudinal_velocity_mps, 0.0);
}

TEST_F(NodeFallbackTest, NearbyAndFastVehiclesRemainVisibleToFreeRunFollowing) {
  const auto base = desired();
  nav_msgs::msg::Odometry ego;
  ego.pose.pose.orientation.w = 1.0;
  BrainInputs inputs;
  ObservedVehicle near;
  near.id = "near"; near.x_m = 1.5; near.vx_mps = 6.9;
  inputs.opponents = {near};
  const auto leading = node.selectFollowVehicle(base.trajectory, base.trajectory, ego, inputs);
  ASSERT_TRUE(leading);
  EXPECT_EQ(leading->vehicle.id, "near");
  EXPECT_FLOAT_EQ(node.makeBrainFollowCommand(base, leading, node.driving_fsm_)
      .trajectory.points.front().longitudinal_velocity_mps, 6.9F);
  inputs.opponents.front().y_m = 3.0;
  EXPECT_FALSE(node.selectFollowVehicle(base.trajectory, base.trajectory, ego, inputs));
}

TEST_F(NodeFallbackTest, RoadCandidatesHaveNoDesignatedTargetAndKeepAllObstacles) {
  auto global = std::make_shared<Trajectory>(desired().trajectory);
  auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.resolution = 0.25; map->info.width = map->info.height = 400;
  map->info.origin.position.x = map->info.origin.position.y = -40.0;
  map->data.assign(160000, 0);
  BrainInputs inputs; inputs.base_reference = global; inputs.wall_map = map;
  inputs.wall_index = std::make_shared<OccupancyGridWallIndex>(map);
  nav_msgs::msg::Odometry ego;
  ego.header.stamp.sec = 122; ego.pose.pose.orientation.w = 1; ego.twist.twist.linear.x = 4;
  ObservedVehicle a; a.id = "a"; a.x_m = 8; a.y_m = 0.5; a.vx_mps = 6.9;
  ObservedVehicle b = a; b.id = "b"; b.x_m = 13; b.y_m = -1.5;
  inputs.opponents = {a, b};
  const auto left = node.makeBrainWork(desired(), ego, inputs, 1);
  const auto right = node.makeBrainWork(desired(), ego, inputs, -1);
  ASSERT_TRUE(left); ASSERT_TRUE(right);
  EXPECT_EQ(left->request.dynamic_obstacle_count, 2U);
  EXPECT_EQ(right->request.dynamic_obstacle_count, 2U);
  EXPECT_EQ(left->request.overtake_target_index, std::numeric_limits<std::size_t>::max());
  EXPECT_DOUBLE_EQ(left->request.pass_profile_origin_d_m, 0.0);
  EXPECT_GT(left->request.nominal.d_pass_m, 0.0);
  EXPECT_LT(right->request.nominal.d_pass_m, 0.0);
  inputs.opponents.front().y_m = 2.0;
  inputs.opponents.front().id = "different";
  const auto changed = node.makeBrainWork(desired(), ego, inputs, 1);
  ASSERT_TRUE(changed);
  EXPECT_DOUBLE_EQ(changed->request.nominal.d_pass_m, left->request.nominal.d_pass_m);
  EXPECT_EQ(changed->request.semantic_key, left->request.semantic_key);
  for (std::size_t i = 0; i < left->request.base_reference_count; ++i) {
    EXPECT_DOUBLE_EQ(changed->request.base_reference[i].pass_d_m, left->request.base_reference[i].pass_d_m);
  }
  const auto returning = node.makeBrainWork(desired(), ego, inputs, 1, mppi::Phase::MERGE);
  ASSERT_TRUE(returning);
  EXPECT_DOUBLE_EQ(returning->request.nominal.d_pass_m, 0.0);
  EXPECT_EQ(returning->request.dynamic_obstacle_count, 2U);
}

TEST_F(NodeFallbackTest, HeartbeatCannotCommitAcrossDrivingModeTransition) {
  BrainInputs snapshot;
  node.driving_fsm_.acceptLateral();
  EXPECT_FALSE(node.publishHeartbeatCommand(snapshot, desired()));
  snapshot.driving_fsm = node.driving_fsm_;
  EXPECT_TRUE(node.publishHeartbeatCommand(snapshot, desired()));
}

TEST_F(NodeFallbackTest, ObstructedReferenceQueuesFiveLateralLinesWithoutEnteringAvoid) {
  const auto command = prepareBrainScene();
  ObservedVehicle lead;
  lead.id = "slow_lead"; lead.x_m = 6.7; lead.vx_mps = 4.;
  lead.stamp_sec = lead.receive_sec = node.now().seconds();
  node.observed_vehicles_[lead.id] = lead;
  node.receiveBrainCommand(command);
  ASSERT_TRUE(node.pending_batch_);
  const auto &batch = *node.pending_batch_;
  EXPECT_EQ(node.driving_fsm_.mode(), DrivingMode::FREE_RUN);
  EXPECT_FALSE(node.active_brain_command_);
  ASSERT_EQ(batch.candidate_count, 5U);
  int left = 0, right = 0;
  for (const auto &candidate : batch.candidates) {
    EXPECT_EQ(candidate.request.dynamic_obstacle_count, 1U);
    EXPECT_EQ(candidate.request.overtake_target_index, std::numeric_limits<std::size_t>::max());
    EXPECT_NE(candidate.request.nominal.d_pass_m, 0.0);
    if (candidate.request.nominal.d_pass_m > 0) ++left; else ++right;
  }
  EXPECT_GT(left, 0); EXPECT_GT(right, 0);
}

TEST_F(NodeFallbackTest, AvoidReturnsOnlyAfterReferenceIsClearAndAligned) {
  const auto command = prepareBrainScene(0.5);
  node.driving_fsm_.acceptLateral();
  node.active_brain_command_ = command;
  node.active_brain_command_->mode = "AVOID";
  node.receiveBrainCommand(command);
  EXPECT_EQ(node.driving_fsm_.mode(), DrivingMode::AVOID);
  ASSERT_TRUE(node.pending_batch_);
  ASSERT_EQ(node.pending_batch_->candidate_count, 1U);
  EXPECT_EQ(node.pending_batch_->candidates.front().request.phase, mppi::Phase::MERGE);
  EXPECT_EQ(node.pending_batch_->candidates.front().request.nominal.d_pass_m, 0.0);
  node.pending_batch_.reset();
  node.odometry_->pose.pose.position.y = 0.0;
  node.receiveBrainCommand(command);
  EXPECT_EQ(node.driving_fsm_.mode(), DrivingMode::FREE_RUN);
  EXPECT_FALSE(node.active_brain_command_);
  EXPECT_FALSE(node.pending_batch_);
  EXPECT_EQ(node.latest_semantic_key_.load(), 0U);
}

TEST_F(NodeFallbackTest, GentleEntryKeepsFiveLinesAndSpeedSearch) {
  const auto command=prepareBrainScene();
  node.gentle_lateral_acceleration_mps2_=1.5;
  node.odometry_->twist.twist.linear.x=8.0;
  ObservedVehicle lead;
  lead.id="lead";lead.x_m=6.7;lead.vx_mps=4.;
  lead.stamp_sec=lead.receive_sec=node.now().seconds();
  node.observed_vehicles_[lead.id]=lead;
  node.receiveBrainCommand(command);
  ASSERT_TRUE(node.pending_batch_);
  ASSERT_EQ(node.pending_batch_->candidate_count,5U);
  for (const auto &work:node.pending_batch_->candidates) {
    const auto &r=work.request;
    EXPECT_GT(r.bounds.minimum.l_out_m,12.0);
    EXPECT_LE(r.bounds.minimum.l_out_m,r.bounds.maximum.l_out_m);
    EXPECT_NEAR(r.bounds.minimum.lateral_control_near_scale,2.0/7.0,1e-12);
    EXPECT_NEAR(r.bounds.maximum.lateral_control_far_scale,5.0/7.0,1e-12);
    EXPECT_DOUBLE_EQ(r.bounds.minimum.speed_scale,0.0);
    EXPECT_DOUBLE_EQ(r.bounds.maximum.speed_scale,1.0);
    EXPECT_EQ(r.overtake_target_index,std::numeric_limits<std::size_t>::max());
    EXPECT_TRUE(r.rollout_constraint_validator);
  }
  auto returning=node.pending_batch_->candidates.front().request;
  returning.phase=mppi::Phase::MERGE;
  const auto before=returning.nominal;
  node.applyGentleLateralEntry(returning);
  EXPECT_DOUBLE_EQ(returning.nominal.l_merge_m,before.l_merge_m);
}

TEST_F(NodeFallbackTest, CrossingReferenceDoesNotFinishAnUnreturnedLateralExecution) {
  const auto command=prepareBrainScene();
  node.compare_return_continuation_=true;
  node.driving_fsm_.acceptLateral();
  node.active_brain_command_=command;
  node.active_brain_command_->corridor_side=1;
  node.receiveBrainCommand(command);
  EXPECT_EQ(node.driving_fsm_.mode(),DrivingMode::AVOID);
  ASSERT_TRUE(node.pending_batch_);
  EXPECT_EQ(node.pending_batch_->candidate_count,1U);
  EXPECT_EQ(node.pending_batch_->candidates[0].request.phase,mppi::Phase::MERGE);
  ASSERT_TRUE(node.pending_batch_->brain_hold);
  EXPECT_EQ(node.pending_batch_->brain_hold->corridor_side,1);
  node.pending_batch_.reset();
  node.active_brain_command_->corridor_side=0;
  node.receiveBrainCommand(command);
  EXPECT_EQ(node.driving_fsm_.mode(),DrivingMode::FREE_RUN);
}

TEST_F(NodeFallbackTest, WorkerCommitsAvoidOnlyWithAcceptedLateralExecution) {
  const auto command = prepareBrainScene();
  ObservedVehicle lead;
  lead.id = "slow_lead"; lead.x_m = 6.7; lead.vx_mps = 4.;
  lead.stamp_sec = lead.receive_sec = node.now().seconds();
  node.observed_vehicles_[lead.id] = lead;
  node.receiveBrainCommand(command);
  ASSERT_TRUE(node.pending_batch_);
  ASSERT_EQ(node.driving_fsm_.mode(), DrivingMode::FREE_RUN);
  node.batch_optimizer_ = std::make_unique<mppi::ReferenceSpaceMppiBatchOptimizer>(node.config_, false);
  node.stopping_ = false;
  node.worker_ = std::thread([&] { node.workerLoop(); });
  bool accepted = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(node.input_mutex_);
      accepted = node.active_brain_command_.has_value();
    }
    if (accepted) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  stopBrainWorker();
  ASSERT_TRUE(accepted);
  EXPECT_EQ(node.driving_fsm_.mode(), DrivingMode::AVOID);
  EXPECT_EQ(node.active_brain_command_->mode, "AVOID");
  EXPECT_NE(node.active_brain_command_->corridor_side, 0);
}

TEST_F(NodeFallbackTest, AcceptedAvoidanceKeepsFollowingReleaseInSourceProfile) {
  const auto command=prepareBrainScene();
  node.config_.longitudinal_planning_enabled=true;
  ObservedVehicle lead;
  lead.id="slow_lead"; lead.x_m=6.7; lead.vx_mps=4.;
  lead.stamp_sec=lead.receive_sec=node.now().seconds();
  node.observed_vehicles_[lead.id]=lead;
  node.receiveBrainCommand(command);
  ASSERT_TRUE(node.pending_batch_);
  node.batch_optimizer_=std::make_unique<mppi::ReferenceSpaceMppiBatchOptimizer>(node.config_,false);
  node.stopping_=false;
  node.worker_=std::thread([&]{node.workerLoop();});
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
  bool accepted=false;
  while(std::chrono::steady_clock::now()<deadline) {
    {
      std::lock_guard<std::mutex> lock(node.input_mutex_);
      accepted=node.active_brain_command_.has_value();
    }
    if(accepted) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  stopBrainWorker();
  ASSERT_TRUE(accepted);
  const auto snapshot=node.captureBrainInputs();
  ASSERT_TRUE(snapshot.active_command); ASSERT_TRUE(snapshot.speed_profile);
  const auto &profile=*snapshot.speed_profile;
  ASSERT_TRUE(std::isfinite(profile.overtake_start_source_s_m));
  const auto source=node.temporaryReferenceFromCommand(*snapshot.active_command);
  ASSERT_EQ(source.count,profile.count);
  for(std::size_t i=0;i<source.count;++i) {
    EXPECT_DOUBLE_EQ(source.points[i].source_s_m,profile.points[i].s_m);
    EXPECT_DOUBLE_EQ(source.points[i].speed_mps,profile.points[i].speed_mps);
  }
  mppi::EgoState ego;
  ego.x_m=source.points[5].x_m; ego.y_m=source.points[5].y_m;
  ego.yaw_rad=source.points[5].yaw_rad;
  const auto remaining=mppi::remainingExecutionTrajectory(source,ego,20.,&profile);
  ASSERT_TRUE(remaining);
  EXPECT_GT(remaining->points[0].source_s_m,0.);
  EXPECT_DOUBLE_EQ(remaining->overtake_start_source_s_m,profile.overtake_start_source_s_m);
  node.clearBrainExecution();
  EXPECT_FALSE(node.captureBrainInputs().speed_profile);
}

TEST_F(NodeFallbackTest, AvoidanceFiveMetreStartUsesBodyGapAndKeepsDistantOpponentObserved) {
  const auto command=prepareBrainScene();
  ObservedVehicle lead;
  lead.id="lead"; lead.vx_mps=4.;
  lead.stamp_sec=rclcpp::Time(node.odometry_->header.stamp).seconds();
  lead.receive_sec=node.now().seconds();
  const double body_extents=node.brain_footprint_front_m_+node.brain_footprint_rear_m_;
  lead.x_m=body_extents+5.001;
  node.observed_vehicles_[lead.id]=lead;
  node.receiveBrainCommand(command);
  EXPECT_FALSE(node.pending_batch_);
  EXPECT_EQ(node.driving_fsm_.mode(),DrivingMode::FREE_RUN);
  const auto inputs=node.captureBrainInputs();
  ASSERT_EQ(inputs.opponents.size(),1U);
  const auto leading=node.selectFollowVehicle(*inputs.base_reference,command.trajectory,
                                              *inputs.odometry,inputs);
  ASSERT_TRUE(leading);
  EXPECT_NEAR(leading->body_gap_m,5.001,1e-6);
  node.observed_vehicles_[lead.id].x_m=body_extents+4.999;
  node.receiveBrainCommand(command);
  ASSERT_TRUE(node.pending_batch_);
  EXPECT_EQ(node.pending_batch_->candidate_count,5U);
  EXPECT_EQ(node.driving_fsm_.mode(),DrivingMode::FREE_RUN);
}

TEST_F(NodeFallbackTest, DistantAvoidanceOnlyForTargetsAtMostFiveKmh) {
  const auto command = prepareBrainScene();
  struct Case { double vx; double vy; bool starts; };
  for (const auto &sample : {
      Case{0.0, 0.0, true}, Case{5.0 / 3.6, 0.0, true},
      Case{std::nextafter(5.0 / 3.6,INFINITY), 0.0, false},
      Case{1.0, 1.0, false}, Case{4.0, 0.0, false},
      Case{20.0 / 3.6, 0.0, false}, Case{20.0 / 3.6 + 1e-6, 0.0, false},
      Case{4.0, 4.0, false}}) {
    SCOPED_TRACE(::testing::Message() << "vx=" << sample.vx << " vy=" << sample.vy);
    node.pending_batch_.reset();
    ObservedVehicle lead;
    lead.id = "lead";
    lead.x_m = node.brain_footprint_front_m_ + node.brain_footprint_rear_m_ + 15.0;
    lead.vx_mps = sample.vx;
    lead.vy_mps = sample.vy;
    lead.stamp_sec = rclcpp::Time(node.odometry_->header.stamp).seconds();
    lead.receive_sec = node.now().seconds();
    node.observed_vehicles_[lead.id] = lead;
    const auto inputs = node.captureBrainInputs();
    const auto leading = node.selectFollowVehicle(*inputs.base_reference, command.trajectory,
                                                  *inputs.odometry, inputs);
    ASSERT_TRUE(leading);
    EXPECT_GT(leading->body_gap_m, 5.0);
    node.receiveBrainCommand(command);
    EXPECT_EQ(node.pending_batch_.has_value(), sample.starts);
    if (sample.starts) {
      ASSERT_TRUE(node.pending_batch_);
      EXPECT_EQ(node.pending_batch_->candidate_count, 5U);
      EXPECT_EQ(node.pending_batch_->maneuver, DrivingMode::AVOID);
      EXPECT_TRUE(node.maneuverCurrent(*node.pending_batch_,node.captureBrainInputs()));
    }
    EXPECT_EQ(node.driving_fsm_.mode(), DrivingMode::FREE_RUN);
  }
}

TEST_F(NodeFallbackTest, EarlyAvoidanceRevalidatesSpeedTargetAndFreshness) {
  const auto command=prepareBrainScene();
  ObservedVehicle lead;lead.id="stopped";lead.x_m=15.;
  lead.stamp_sec=lead.receive_sec=node.now().seconds();
  node.observed_vehicles_[lead.id]=lead;
  node.receiveBrainCommand(command);
  ASSERT_TRUE(node.pending_batch_);
  const auto batch=*node.pending_batch_;
  EXPECT_TRUE(node.maneuverCurrent(batch,node.captureBrainInputs()));
  node.observed_vehicles_[lead.id].vx_mps=5./3.6;
  EXPECT_TRUE(node.maneuverCurrent(batch,node.captureBrainInputs()));
  node.observed_vehicles_[lead.id].vx_mps=std::nextafter(5./3.6,INFINITY);
  EXPECT_FALSE(node.maneuverCurrent(batch,node.captureBrainInputs()));
  node.observed_vehicles_[lead.id].x_m=node.brain_footprint_front_m_+
      node.brain_footprint_rear_m_+4.;
  EXPECT_TRUE(node.maneuverCurrent(batch,node.captureBrainInputs()));
  node.observed_vehicles_[lead.id]=lead;
  node.observed_vehicles_[lead.id].stamp_sec-=node.brain_input_timeout_sec_+1.;
  node.observed_vehicles_[lead.id].receive_sec-=node.brain_input_timeout_sec_+1.;
  EXPECT_FALSE(node.maneuverCurrent(batch,node.captureBrainInputs()));
  node.observed_vehicles_.clear();lead.id="replacement";
  node.observed_vehicles_[lead.id]=lead;
  EXPECT_FALSE(node.maneuverCurrent(batch,node.captureBrainInputs()));
}

TEST_F(NodeFallbackTest, EarlyAvoidanceRetainsExistingForwardRangeAndPathConflict) {
  auto command=prepareBrainScene();
  node.brain_trigger_distance_m_=40.;
  for(int i=80;i<160;++i) {
    auto point=command.trajectory.points.back();point.pose.position.x=i*.5;
    command.trajectory.points.push_back(point);
  }
  node.base_reference_=std::make_shared<Trajectory>(command.trajectory);
  struct Case { double x; double y; bool selected; };
  for(const auto &sample:{Case{40.,0.,true},Case{40.001,0.,false},
                         Case{-3.,0.,false},Case{15.,4.,false}}) {
    SCOPED_TRACE(::testing::Message()<<sample.x<<" "<<sample.y);
    ObservedVehicle lead;lead.id="parked";lead.x_m=sample.x;lead.y_m=sample.y;
    lead.stamp_sec=lead.receive_sec=node.now().seconds();
    node.observed_vehicles_[lead.id]=lead;
    const auto inputs=node.captureBrainInputs();
    EXPECT_EQ(inputs.leading.has_value(),sample.selected);
  }
}

TEST_F(NodeFallbackTest, EarlyAvoidanceContinuesButDoesNotInterruptCommittedReturn) {
  const auto command=prepareBrainScene(.5);
  node.compare_return_continuation_=true;
  node.driving_fsm_.acceptLateral();
  node.active_brain_command_=command;node.active_brain_command_->corridor_side=1;
  ObservedVehicle lead;lead.id="parked";lead.x_m=15.;
  lead.stamp_sec=lead.receive_sec=node.now().seconds();node.observed_vehicles_[lead.id]=lead;
  node.receiveBrainCommand(command);
  ASSERT_TRUE(node.pending_batch_);
  EXPECT_EQ(node.pending_batch_->candidate_count,5U);
  EXPECT_TRUE(node.pending_batch_->allow_continuation);
  EXPECT_EQ(node.pending_batch_->candidates[0].request.phase,mppi::Phase::OVERTAKE);
  EXPECT_EQ(node.pending_batch_->maneuver,DrivingMode::AVOID);
  node.active_brain_command_->corridor_side=0;
  node.pending_batch_.reset();
  node.receiveBrainCommand(command);
  ASSERT_TRUE(node.pending_batch_);
  EXPECT_EQ(node.pending_batch_->candidate_count,1U);
  EXPECT_EQ(node.pending_batch_->candidates[0].request.phase,mppi::Phase::MERGE);
}

TEST_F(NodeFallbackTest, FastLeaderInsideFiveMetresFollowsUntilPreparationExists) {
  const auto command=prepareBrainScene();
  node.brain_cruise_speed_mps_=10.;
  for(double gap:{5.,3.,2.,1.9}) {
    ObservedVehicle lead;lead.id="fast";lead.vx_mps=7.;
    lead.x_m=node.brain_footprint_front_m_+node.brain_footprint_rear_m_+gap;
    lead.stamp_sec=lead.receive_sec=node.now().seconds();
    node.observed_vehicles_[lead.id]=lead;
    node.receiveBrainCommand(command);
    EXPECT_FALSE(node.pending_batch_);
    EXPECT_EQ(node.driving_fsm_.mode(),DrivingMode::FREE_RUN);
    const auto follow=followAt(gap,"fast",7.);
    EXPECT_FLOAT_EQ(follow.trajectory.points[0].longitudinal_velocity_mps,gap>2.?10.:7.);
  }
}

TEST_F(NodeFallbackTest, ActiveAvoidanceOutsideFiveMetresQueuesOnlyReturnWithoutOldPathRanking) {
  const auto command=prepareBrainScene(.5);
  node.compare_return_continuation_=true;
  node.driving_fsm_.acceptLateral();
  node.active_brain_command_=command;node.active_brain_command_->corridor_side=1;
  ObservedVehicle lead;lead.id="slow";lead.x_m=15.;lead.vx_mps=4.;
  lead.stamp_sec=lead.receive_sec=node.now().seconds();node.observed_vehicles_[lead.id]=lead;
  node.receiveBrainCommand(command);
  ASSERT_TRUE(node.pending_batch_);
  ASSERT_EQ(node.pending_batch_->candidate_count,1U);
  EXPECT_EQ(node.pending_batch_->candidates[0].request.phase,mppi::Phase::MERGE);
  EXPECT_FALSE(node.pending_batch_->allow_continuation);
  EXPECT_FALSE(node.pending_batch_->candidates[0].request.passing_preparation);
}

TEST_F(NodeFallbackTest, AvoidanceBatchRevalidationRejectsDistanceSpeedAndTargetChanges) {
  const auto command=prepareBrainScene();
  ObservedVehicle lead;lead.id="slow";lead.x_m=6.7;lead.vx_mps=4.;
  lead.stamp_sec=lead.receive_sec=node.now().seconds();node.observed_vehicles_[lead.id]=lead;
  node.receiveBrainCommand(command);
  ASSERT_TRUE(node.pending_batch_);
  const auto batch=*node.pending_batch_;
  EXPECT_TRUE(node.maneuverCurrent(batch,node.captureBrainInputs()));
  node.observed_vehicles_[lead.id].x_m=15.;
  EXPECT_FALSE(node.maneuverCurrent(batch,node.captureBrainInputs()));
  node.observed_vehicles_[lead.id]=lead;node.observed_vehicles_[lead.id].vx_mps=7.;
  EXPECT_FALSE(node.maneuverCurrent(batch,node.captureBrainInputs()));
  node.observed_vehicles_.clear();lead.id="replacement";node.observed_vehicles_[lead.id]=lead;
  EXPECT_FALSE(node.maneuverCurrent(batch,node.captureBrainInputs()));
}

TEST_F(NodeFallbackTest, SlowTargetDropsStalePreparationBeforeAvoidanceWork) {
  const auto command=prepareBrainScene();
  node.passing_preparation_enabled_=true;
  auto plan=std::make_shared<mppi::PassingPreparationPlan>();plan->target_id="slow";
  node.latest_preparation_=plan;
  ObservedVehicle lead;lead.id="slow";lead.x_m=6.7;lead.vx_mps=4.;
  lead.stamp_sec=lead.receive_sec=node.now().seconds();node.observed_vehicles_[lead.id]=lead;
  node.receiveBrainCommand(command);
  EXPECT_FALSE(node.latest_preparation_);
  ASSERT_TRUE(node.pending_batch_);ASSERT_EQ(node.pending_batch_->candidate_count,5U);
  for(const auto &work:node.pending_batch_->candidates) {
    EXPECT_EQ(work.maneuver,DrivingMode::AVOID);
    EXPECT_FALSE(work.request.passing_preparation);
    EXPECT_FALSE(work.request.leader_passing_prediction);
  }
}

TEST_F(NodeFallbackTest, PassingTargetUsesObservedLeadingVehicleAndRetainsAdoptedTarget) {
  prepareBrainScene();
  node.leader_vehicle_id_="upstream";node.leader_receive_sec_=node.now().seconds();
  ObservedVehicle target;target.id="actual";target.x_m=10.;target.vx_mps=7.;
  target.stamp_sec=target.receive_sec=node.now().seconds();node.observed_vehicles_[target.id]=target;
  auto inputs=node.captureBrainInputs();
  EXPECT_EQ(inputs.leader_vehicle_id,"actual");EXPECT_TRUE(node.overtakeTargetValid(inputs));
  node.driving_fsm_.acceptLateral(DrivingMode::OVERTAKE);node.active_maneuver_target_id_="actual";
  target.id="nearer";target.x_m=6.;node.observed_vehicles_[target.id]=target;
  inputs=node.captureBrainInputs();
  ASSERT_TRUE(inputs.leading);EXPECT_EQ(inputs.leading->vehicle.id,"nearer");
  EXPECT_EQ(inputs.leader_vehicle_id,"actual");
}

TEST_F(NodeFallbackTest, LeaderLapGuidancePreservesCurrentMotionCollisionPrediction) {
  prepareBrainScene();
  auto course=std::make_shared<Trajectory>();
  for(int i=0;i<=400;++i){
    autoware_auto_planning_msgs::msg::TrajectoryPoint p;
    const double a=2.*M_PI*i/400.;
    p.pose.position.x=20.*std::cos(a);p.pose.position.y=20.*std::sin(a);
    p.pose.orientation.w=1.;p.longitudinal_velocity_mps=8.;course->points.push_back(p);
  }
  node.base_reference_=course;
  const auto world=node.referencePoseIndex(*course);
  auto history=std::make_shared<std::vector<opponent_prediction::PositionObservation>>();
  auto online=std::make_shared<opponent_prediction::OnlineMotionLearner>();
  const double now=node.now().seconds();
  for(int i=0;i<=1400;++i){
    const double t=i*.02,a=2.*M_PI*t/20.;
    const double x=20.*std::cos(a),y=20.*std::sin(a),s=*world.projectStation(x,y);
    history->push_back({now+t-28.,x,y,history->empty()?s:history->back().station+world.stationDifference(s,history->back().station)});
    online->observe(*history,&world);
  }
  ObservedVehicle lead;
  lead.id="lead";lead.x_m=history->back().x;lead.y_m=history->back().y;
  lead.history=history;lead.online_motion=online;
  lead.stamp_sec=lead.receive_sec=now;node.observed_vehicles_[lead.id]=lead;
  node.odometry_->header.stamp=rclcpp::Time(static_cast<int64_t>(now*1e9));
  const auto ego_pose=world.pose(history->back().station-10.,0.);
  ASSERT_TRUE(ego_pose);
  node.odometry_->pose.pose.position.x=ego_pose->at(0);
  node.odometry_->pose.pose.position.y=ego_pose->at(1);
  node.odometry_->pose.pose.orientation=quaternionFromYaw(ego_pose->at(2));
  node.leader_vehicle_id_=lead.id;node.leader_receive_sec_=now;
  node.online_motion_prediction_enabled_=true;node.leader_lap_prediction_enabled_=false;
  const auto baseline=node.captureBrainInputs();
  node.leader_lap_prediction_enabled_=true;
  const auto with_guidance=node.captureBrainInputs();
  ASSERT_EQ(baseline.opponents.size(),1U);ASSERT_EQ(with_guidance.opponents.size(),1U);
  const auto &before=baseline.opponents[0],&after=with_guidance.opponents[0];
  ASSERT_TRUE(before.prediction);ASSERT_TRUE(after.prediction);ASSERT_TRUE(after.leader_lap_prediction);
  EXPECT_NE(after.prediction,after.leader_lap_prediction);
  EXPECT_DOUBLE_EQ(before.x_m,after.x_m);EXPECT_DOUBLE_EQ(before.y_m,after.y_m);
  ASSERT_EQ(before.prediction->points.size(),after.prediction->points.size());
  for(size_t i=0;i<before.prediction->points.size();++i){
    const auto &a=before.prediction->points[i],&b=after.prediction->points[i];
    EXPECT_DOUBLE_EQ(a.global_s,b.global_s);EXPECT_DOUBLE_EQ(a.d,b.d);
    EXPECT_DOUBLE_EQ(a.relative_yaw,b.relative_yaw);
  }
  mppi::PlanRequest request;
  const auto ego=node.projectOnTrajectory(*course,ego_pose->at(0),ego_pose->at(1));
  ASSERT_TRUE(node.populateDynamicObstacles(request,with_guidance,ego,node.trajectoryArcLength(*course),true));
  ASSERT_EQ(request.dynamic_obstacle_count,1U);
  EXPECT_EQ(request.dynamic_obstacles[0].prediction,after.prediction);
  EXPECT_EQ(request.leader_passing_prediction,after.leader_lap_prediction);
}

class NodeMatchedPriorTest : public NodeFallbackTest {
protected:
  void scene() {
    prepareBrainScene();
    auto course=std::make_shared<Trajectory>();
    for(int i=0;i<=400;++i) {
      autoware_auto_planning_msgs::msg::TrajectoryPoint p;
      const double a=2.*M_PI*i/400.;
      p.pose.position.x=20.*std::cos(a);p.pose.position.y=20.*std::sin(a);
      p.pose.orientation=quaternionFromYaw(a+M_PI/2.);
      p.longitudinal_velocity_mps=8.;course->points.push_back(p);
    }
    node.base_reference_=course;
    node.online_motion_prediction_enabled_=true;
    node.leader_lap_prediction_enabled_=true;
    node.passing_preparation_enabled_=true;
    node.front_merge_attack_enabled_=true;
    setHistory();
  }
  void setHistory(double recent_offset=0.,double recent_pace=1.,double lap_period=20.) {
    const auto world=node.referencePoseIndex(*node.base_reference_);
    auto history=std::make_shared<std::vector<opponent_prediction::PositionObservation>>();
    auto online=std::make_shared<opponent_prediction::OnlineMotionLearner>();
    const double now=node.now().seconds();
    for(int i=0;i<=1400;++i) {
      const double t=i*.02,phase=t<=26.?t:26.+(t-26.)*recent_pace;
      const double a=2.*M_PI*phase/lap_period,radius=20.+(t>26.?recent_offset:0.);
      const double x=radius*std::cos(a),y=radius*std::sin(a),s=*world.projectStation(x,y);
      history->push_back({now+t-28.,x,y,history->empty()?s:history->back().station+
          world.stationDifference(s,history->back().station)});
      online->observe(*history,&world);
    }
    ObservedVehicle lead;
    lead.id="lead";lead.x_m=history->back().x;lead.y_m=history->back().y;
    lead.history=history;lead.online_motion=online;
    lead.stamp_sec=lead.source_stamp_sec=lead.receive_sec=now;
    node.observed_vehicles_[lead.id]=lead;
    const auto pose=world.pose(history->back().station-10.,0.);
    ASSERT_TRUE(pose);
    node.odometry_->header.stamp=rclcpp::Time(static_cast<int64_t>(now*1e9));
    node.odometry_->pose.pose.position.x=pose->at(0);node.odometry_->pose.pose.position.y=pose->at(1);
    node.odometry_->pose.pose.orientation=quaternionFromYaw(pose->at(2));
  }
};

TEST_F(NodeMatchedPriorTest, MatchingLeaderSharesPredictionAcrossExecutionAndGuidance) {
  scene();
  const auto inputs=node.captureBrainInputs();
  ASSERT_EQ(inputs.opponents.size(),1U);
  const auto &lead=inputs.opponents.front();
  ASSERT_EQ(inputs.leader_vehicle_id,"lead");ASSERT_TRUE(lead.prior_lap_match.usable);
  ASSERT_TRUE(lead.matched_prior_lap_execution);ASSERT_TRUE(lead.prediction);
  EXPECT_EQ(lead.prediction,lead.leader_lap_prediction);
  const auto context=node.executionContext(inputs,*inputs.odometry);
  ASSERT_TRUE(context);ASSERT_EQ(context->dynamic_obstacle_count,1U);
  EXPECT_EQ(context->dynamic_obstacles[0].prediction,lead.prediction);
  EXPECT_EQ(context->leader_passing_prediction,lead.prediction);
  const auto world=node.referencePoseIndex(*inputs.base_reference);
  const auto first=world.pose(lead.prediction->points.front().global_s,lead.prediction->points.front().d);
  ASSERT_TRUE(first);
  EXPECT_NEAR(first->at(0),lead.history->back().x,.01);
  EXPECT_NEAR(first->at(1),lead.history->back().y,.01);
  mppi::PlanRequest collision_only;
  const auto projection=node.projectOnTrajectory(*inputs.base_reference,
      inputs.odometry->pose.pose.position.x,inputs.odometry->pose.pose.position.y);
  ASSERT_TRUE(node.populateDynamicObstacles(collision_only,inputs,projection,
      node.trajectoryArcLength(*inputs.base_reference),true,false));
  EXPECT_EQ(collision_only.dynamic_obstacles[0].prediction,lead.prediction);
}

TEST_F(NodeMatchedPriorTest, RouteMismatchReturnsToCurrentModelWithoutChangingOldSnapshot) {
  scene();const auto matched=node.captureBrainInputs();
  ASSERT_TRUE(matched.opponents.front().matched_prior_lap_execution);
  const auto old=matched.opponents.front().prediction;
  setHistory(.9);
  const auto changed=node.captureBrainInputs();const auto &lead=changed.opponents.front();
  ASSERT_TRUE(lead.prediction);ASSERT_TRUE(lead.leader_lap_prediction);
  EXPECT_STREQ(lead.prior_lap_match.reason,"route_mismatch");
  EXPECT_FALSE(lead.matched_prior_lap_execution);EXPECT_NE(lead.prediction,lead.leader_lap_prediction);
  EXPECT_EQ(matched.opponents.front().prediction,old);
  setHistory();EXPECT_TRUE(node.captureBrainInputs().opponents.front().matched_prior_lap_execution);
}

TEST_F(NodeMatchedPriorTest, PaceMismatchReturnsToCurrentModel) {
  scene();setHistory(0.,1.6);
  const auto inputs=node.captureBrainInputs();const auto &lead=inputs.opponents.front();
  ASSERT_TRUE(lead.prediction);ASSERT_TRUE(lead.leader_lap_prediction);
  EXPECT_STREQ(lead.prior_lap_match.reason,"pace_mismatch");
  EXPECT_FALSE(lead.matched_prior_lap_execution);EXPECT_NE(lead.prediction,lead.leader_lap_prediction);
}

TEST_F(NodeMatchedPriorTest, MissingLapAndDisabledFeatureKeepCurrentModel) {
  scene();node.front_merge_attack_enabled_=false;
  auto inputs=node.captureBrainInputs();
  EXPECT_TRUE(inputs.opponents.front().prior_lap_match.usable);
  EXPECT_FALSE(inputs.opponents.front().matched_prior_lap_execution);
  node.front_merge_attack_enabled_=true;
  auto &lead=node.observed_vehicles_.at("lead");
  auto short_history=std::make_shared<std::vector<opponent_prediction::PositionObservation>>(
      lead.history->end()-50,lead.history->end());lead.history=short_history;
  inputs=node.captureBrainInputs();
  ASSERT_TRUE(inputs.opponents.front().prediction);
  EXPECT_FALSE(inputs.opponents.front().leader_lap_prediction);
  EXPECT_FALSE(inputs.opponents.front().matched_prior_lap_execution);
}

TEST_F(NodeMatchedPriorTest, NonTargetDoesNotInheritMatchedPrediction) {
  scene();
  ObservedVehicle other;other.id="other";other.x_m=30.;other.y_m=30.;
  other.stamp_sec=other.receive_sec=node.now().seconds();
  auto prediction=std::make_shared<opponent_prediction::Prediction>();
  prediction->points={{0.,5.,8.,0.},{6.,10.,8.,0.}};other.prediction=prediction;
  node.observed_vehicles_[other.id]=other;
  const auto inputs=node.captureBrainInputs();ASSERT_EQ(inputs.opponents.size(),2U);
  EXPECT_EQ(inputs.leader_vehicle_id,"lead");
  EXPECT_TRUE(inputs.opponents[0].matched_prior_lap_execution);
  EXPECT_FALSE(inputs.opponents[1].matched_prior_lap_execution);
  EXPECT_EQ(inputs.opponents[1].prediction,prediction);
}

TEST_F(NodeMatchedPriorTest, MatchedPredictionStillRejectsPhysicalCollisionAndWall) {
  scene();const auto inputs=node.captureBrainInputs();
  ASSERT_TRUE(inputs.opponents.front().matched_prior_lap_execution);
  const auto &prediction=*inputs.opponents.front().prediction;
  const auto world=node.referencePoseIndex(*inputs.base_reference);
  const auto p=prediction.at(.5);const auto pose=world.pose(p.global_s,p.d);ASSERT_TRUE(pose);
  mppi::RolloutState a,b;
  a.x_m=b.x_m=pose->at(0);a.y_m=b.y_m=pose->at(1);a.yaw_rad=b.yaw_rad=pose->at(2)+p.relative_yaw;
  a.time_sec=.49;b.time_sec=.5;
  const auto validator=node.makeBrainRolloutConstraintValidator(inputs);
  EXPECT_EQ(validator(a,b),mppi::RejectReason::COLLISION);
  a.x_m=b.x_m=1000.;a.y_m=b.y_m=1000.;
  EXPECT_EQ(validator(a,b),mppi::RejectReason::WALL);
}

TEST_F(NodeMatchedPriorTest, ActiveTargetSwitchReleasesPreviousLeadersPriorModel) {
  scene();const auto previous=node.captureBrainInputs();
  ASSERT_TRUE(previous.opponents.front().matched_prior_lap_execution);
  auto other=node.observed_vehicles_.at("lead");other.id="other";
  node.observed_vehicles_[other.id]=other;
  node.driving_fsm_.acceptLateral(DrivingMode::OVERTAKE);
  node.active_maneuver_target_id_=other.id;
  const auto next=node.captureBrainInputs();
  ASSERT_EQ(next.opponents.size(),2U);ASSERT_EQ(next.leader_vehicle_id,"other");
  EXPECT_FALSE(next.opponents[0].matched_prior_lap_execution);
  EXPECT_FALSE(next.opponents[0].leader_lap_prediction);
  EXPECT_TRUE(next.opponents[1].matched_prior_lap_execution);
  EXPECT_EQ(next.opponents[1].prediction,next.opponents[1].leader_lap_prediction);
}

TEST_F(NodeMatchedPriorTest, SlowAvoidanceTargetKeepsCurrentModelDespiteMatchingLap) {
  scene();setHistory(0.,1.,25.);
  const auto inputs=node.captureBrainInputs();ASSERT_EQ(inputs.leader_vehicle_id,"lead");
  const auto &lead=inputs.opponents.front();
  ASSERT_TRUE(lead.prior_lap_match.usable);ASSERT_TRUE(lead.leader_lap_prediction);
  ASSERT_TRUE(isAvoidanceSpeed(std::hypot(lead.vx_mps,lead.vy_mps)));
  EXPECT_FALSE(node.overtakeTargetValid(inputs));
  EXPECT_FALSE(lead.matched_prior_lap_execution);
  EXPECT_NE(lead.prediction,lead.leader_lap_prediction);
}

#include "target_retention_tests.inc"

} // namespace reference_space_mppi_planner
