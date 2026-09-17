class NodeTargetRetentionTest : public NodeFallbackTest {
protected:
  void scene() {
    prepareBrainScene();node.odometry_->pose.pose.position.x=10.;
    observe("held",15.,0.);
    ASSERT_EQ(node.captureBrainInputs(true).leader_vehicle_id,"held");
  }
  void observe(const std::string &id,double x,double y) {
    ObservedVehicle v;v.id=id;v.x_m=x;v.y_m=y;v.vx_mps=7.;
    v.stamp_sec=v.receive_sec=node.now().seconds();node.observed_vehicles_[id]=v;
  }
};

TEST_F(NodeTargetRetentionTest, FreeRunRetainsTargetAcrossCenterCrossoverAndLateralExit) {
  scene();const auto revision=node.execution_revision_;
  observe("held",15.,3.);node.odometry_->pose.pose.position.x=15.25;
  const auto inputs=node.captureBrainInputs(true);
  EXPECT_EQ(inputs.driving_fsm.mode(),DrivingMode::FREE_RUN);
  EXPECT_FALSE(inputs.leading);EXPECT_EQ(inputs.leader_vehicle_id,"held");
  EXPECT_EQ(inputs.prediction_target_id,"held");
  EXPECT_LT(node.targetBodyPassGap(inputs,"held"),0.);
  EXPECT_EQ(node.execution_revision_,revision);
}

TEST_F(NodeTargetRetentionTest, BodyClearanceReleasesTargetAndInvalidatesOldSnapshot) {
  scene();const auto old=node.captureBrainInputs(true);
  node.odometry_->pose.pose.position.x=20.;
  const auto cleared=node.captureBrainInputs(true);
  EXPECT_GT(node.targetBodyPassGap(cleared,"held"),0.);
  EXPECT_TRUE(cleared.leader_vehicle_id.empty());EXPECT_TRUE(node.prediction_target_id_.empty());
  EXPECT_GT(cleared.execution_revision,old.execution_revision);
  EXPECT_FALSE(node.heartbeatSnapshotCurrent(old));
}

TEST_F(NodeTargetRetentionTest, NextFrontCarCannotReplaceAnOverlappingBody) {
  scene();observe("held",15.,3.);observe("next",25.,0.);
  node.odometry_->pose.pose.position.x=15.25;
  auto inputs=node.captureBrainInputs(true);
  ASSERT_TRUE(inputs.leading);EXPECT_EQ(inputs.leading->vehicle.id,"next");
  EXPECT_EQ(inputs.leader_vehicle_id,"held");EXPECT_EQ(inputs.opponents.size(),2U);
  node.odometry_->pose.pose.position.x=20.;
  inputs=node.captureBrainInputs(true);
  EXPECT_EQ(inputs.leader_vehicle_id,"next");EXPECT_EQ(inputs.opponents.size(),2U);
}

TEST_F(NodeTargetRetentionTest, NewImmediateFrontCarTakesOverBeforeManeuverAdoption) {
  scene();observe("held",25.,0.);observe("replacement",18.,0.);
  const auto old=node.captureBrainInputs();
  const auto inputs=node.captureBrainInputs(true);
  EXPECT_EQ(inputs.leader_vehicle_id,"replacement");EXPECT_EQ(node.prediction_target_id_,"replacement");
  EXPECT_EQ(inputs.opponents.size(),2U);
  EXPECT_GT(inputs.execution_revision,old.execution_revision);
}

TEST_F(NodeTargetRetentionTest, AcquisitionDistanceDoesNotReleaseAnExistingTarget) {
  scene();node.brain_trigger_distance_m_=10.;observe("held",25.,0.);
  const auto inputs=node.captureBrainInputs(true);
  EXPECT_FALSE(inputs.leading);EXPECT_EQ(inputs.leader_vehicle_id,"held");
  EXPECT_EQ(inputs.opponents.size(),1U);
}

TEST_F(NodeTargetRetentionTest, MissingObservationKeepsIdentityWithoutUsingStalePosition) {
  scene();node.observed_vehicles_.at("held").stamp_sec=-2.;
  node.observed_vehicles_.at("held").receive_sec=-2.;
  auto inputs=node.captureBrainInputs(true);
  EXPECT_EQ(inputs.leader_vehicle_id,"held");EXPECT_TRUE(inputs.opponents.empty());
  EXPECT_TRUE(std::isnan(node.targetBodyPassGap(inputs,"held")));
  observe("held",15.,3.);node.odometry_->pose.pose.position.x=15.25;
  inputs=node.captureBrainInputs(true);EXPECT_EQ(inputs.leader_vehicle_id,"held");
  EXPECT_EQ(inputs.opponents.size(),1U);
}

TEST_F(NodeTargetRetentionTest, ActiveManeuverAndReturnKeepTargetUntilCompletion) {
  scene();node.driving_fsm_.acceptLateral(DrivingMode::OVERTAKE);
  node.active_maneuver_target_id_="held";
  node.active_brain_command_=desired();node.active_brain_command_->corridor_side=1;
  node.odometry_->pose.pose.position.x=20.;observe("next",30.,0.);
  EXPECT_EQ(node.captureBrainInputs(true).leader_vehicle_id,"held");
  auto returned=desired();returned.corridor_side=0;node.adoptBrainReturn(returned);
  EXPECT_TRUE(node.active_maneuver_target_id_.empty());
  EXPECT_EQ(node.captureBrainInputs(true).leader_vehicle_id,"held");
  node.driving_fsm_.finishAvoid();node.clearBrainExecution();
  EXPECT_EQ(node.captureBrainInputs(true).leader_vehicle_id,"next");
}

TEST_F(NodeTargetRetentionTest, ActiveMissingTargetDoesNotSwitchToAnotherCar) {
  scene();node.driving_fsm_.acceptLateral(DrivingMode::OVERTAKE);
  node.active_maneuver_target_id_="held";node.observed_vehicles_.erase("held");
  observe("replacement",18.,0.);
  const auto inputs=node.captureBrainInputs(true);
  EXPECT_EQ(inputs.leader_vehicle_id,"held");ASSERT_TRUE(inputs.leading);
  EXPECT_EQ(inputs.leading->vehicle.id,"replacement");
  EXPECT_FALSE(node.overtakeTargetValid(inputs));
}

TEST_F(NodeTargetRetentionTest, ReferenceCommitPreservesTrackedIdentity) {
  scene();auto changed=std::make_shared<Trajectory>(*node.base_reference_);
  for(auto &p:changed->points)p.pose.position.y+=.2;
  node.commitBaseReference(changed);
  EXPECT_EQ(node.prediction_target_id_,"held");
  observe("held",15.,3.);node.odometry_->pose.pose.position.x=15.25;
  EXPECT_EQ(node.captureBrainInputs(true).leader_vehicle_id,"held");
}

TEST_F(NodeTargetRetentionTest, BodyClearanceUsesVehicleHeading) {
  scene();node.brain_footprint_front_m_=1.;node.brain_footprint_rear_m_=1.;
  node.brain_footprint_radius_m_=.5;
  auto inputs=node.captureBrainInputs();
  inputs.opponents[0].x_m=15.;
  inputs.odometry->pose.pose.position.x=16.75;
  EXPECT_NEAR(node.targetBodyPassGap(inputs,"held"),-.25,1e-8);
  inputs.opponents[0].vx_mps=0.;inputs.opponents[0].vy_mps=7.;
  EXPECT_NEAR(node.targetBodyPassGap(inputs,"held"),.25,1e-8);
}

TEST_F(NodeTargetRetentionTest, BodyClearanceAcrossLapBoundary) {
  scene();auto course=std::make_shared<Trajectory>();
  for(int i=0;i<=400;++i) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint p;
    double a=2.*M_PI*i/400.;p.pose.position.x=20.*std::cos(a);p.pose.position.y=20.*std::sin(a);
    p.pose.orientation=quaternionFromYaw(a+M_PI/2.);course->points.push_back(p);
  }
  node.commitBaseReference(course);const auto world=node.sharedReferencePoseIndex(course);
  const auto other=world->pose(node.trajectoryArcLength(*course)-.4,0.);
  ASSERT_TRUE(other);observe("held",other->at(0),other->at(1));
  node.observed_vehicles_["held"].vx_mps=7.*std::cos(other->at(2));
  node.observed_vehicles_["held"].vy_mps=7.*std::sin(other->at(2));
  for(double station:{.2,4.}) {
    const auto pose=world->pose(station,0.);ASSERT_TRUE(pose);
    node.odometry_->pose.pose.position.x=pose->at(0);node.odometry_->pose.pose.position.y=pose->at(1);
    node.odometry_->pose.pose.orientation=quaternionFromYaw(pose->at(2));
    const auto inputs=node.captureBrainInputs(true);
    if(station<1.) {EXPECT_LT(node.targetBodyPassGap(inputs,"held"),0.);EXPECT_EQ(inputs.leader_vehicle_id,"held");}
    else {EXPECT_GT(node.targetBodyPassGap(inputs,"held"),0.);EXPECT_TRUE(inputs.leader_vehicle_id.empty());}
  }
}

TEST_F(NodeTargetRetentionTest, FrontMergeRequiresPhysicalPassEndAndAlignment) {
  scene();auto inputs=node.captureBrainInputs();
  auto plan=std::make_shared<mppi::PassingPreparationPlan>();plan->front_merge=true;
  plan->target_id="held";plan->connection_end_station_m=19.;inputs.active_preparation=plan;
  inputs.odometry->pose.pose.position.x=15.25;
  EXPECT_FALSE(node.frontMergeComplete(inputs,20.,true));
  inputs.odometry->pose.pose.position.x=20.;
  EXPECT_TRUE(node.frontMergeComplete(inputs,20.,true));
  EXPECT_FALSE(node.frontMergeComplete(inputs,18.,true));
  EXPECT_FALSE(node.frontMergeComplete(inputs,20.,false));
  inputs.opponents.clear();EXPECT_FALSE(node.frontMergeComplete(inputs,20.,true));
}

TEST_F(NodeTargetRetentionTest, HeartbeatDoesNotFinishFrontMergeWithOverlappingTarget) {
  scene();node.front_merge_attack_enabled_=true;node.driving_fsm_.acceptLateral(DrivingMode::OVERTAKE);
  auto plan=std::make_shared<mppi::PassingPreparationPlan>();plan->front_merge=true;
  plan->target_id="held";plan->connection_end_station_m=19.;plan->world=node.sharedReferencePoseIndex(node.base_reference_);
  node.active_preparation_=plan;node.active_maneuver_target_id_="held";
  node.active_brain_command_=desired();node.active_brain_command_->corridor_side=1;
  node.odometry_->pose.pose.position.x=20.;observe("held",19.75,3.);
  auto command=desired();command.header.stamp=node.now();
  node.receiveBrainCommand(command);
  EXPECT_EQ(node.driving_fsm_.mode(),DrivingMode::OVERTAKE);
  EXPECT_EQ(node.prediction_target_id_,"held");
  observe("held",15.,3.);
  node.receiveBrainCommand(command);
  EXPECT_EQ(node.driving_fsm_.mode(),DrivingMode::FREE_RUN);
  EXPECT_TRUE(node.captureBrainInputs(true).leader_vehicle_id.empty());
}

TEST_F(NodeMatchedPriorTest, RetainedFreeRunTargetKeepsMatchedModelAcrossCenterCrossover) {
  scene();auto first=node.captureBrainInputs(true);
  ASSERT_TRUE(first.opponents.front().matched_prior_lap_execution);
  const auto &other=node.observed_vehicles_.at("lead");
  const auto world=node.sharedReferencePoseIndex(node.base_reference_);
  const auto ego=world->pose(other.history->back().station+.25,0.);ASSERT_TRUE(ego);
  node.odometry_->pose.pose.position.x=ego->at(0);node.odometry_->pose.pose.position.y=ego->at(1);
  node.odometry_->pose.pose.orientation=quaternionFromYaw(ego->at(2));
  const auto crossed=node.captureBrainInputs(true);
  EXPECT_FALSE(crossed.leading);EXPECT_EQ(crossed.leader_vehicle_id,"lead");
  EXPECT_FALSE(node.overtakeTargetValid(crossed));
  ASSERT_TRUE(crossed.opponents.front().matched_prior_lap_execution);
  EXPECT_EQ(crossed.opponents.front().prediction,crossed.opponents.front().leader_lap_prediction);
  EXPECT_EQ(crossed.driving_fsm.mode(),DrivingMode::FREE_RUN);
}
