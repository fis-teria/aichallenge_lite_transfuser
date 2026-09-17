class NodeReferenceContractTest : public NodeFallbackTest {
protected:
  Command prepare() {
    auto command=prepareBrainScene();
    command.mode="FREE_RUN";
    node.publishOutput(command);
    return command;
  }
  Trajectory::SharedPtr route(double y) {
    auto result=std::make_shared<Trajectory>(desired().trajectory);
    for (auto &p:result->points) p.pose.position.y=y;
    return result;
  }
  void other(double x,double y) {
    ObservedVehicle car;
    car.id="d3";car.x_m=x;car.y_m=y;car.vx_mps=4.;
    car.stamp_sec=car.receive_sec=node.now().seconds();
    node.observed_vehicles_[car.id]=car;
  }
};

TEST_F(NodeReferenceContractTest, RequestArrivalAndRepeatedPublicationDoNotReplaceTheCommittedWorld) {
  prepare();
  const auto old=node.base_reference_;
  const auto before=node.captureBrainInputs();
  node.receiveBaseReference(route(2.));
  EXPECT_EQ(node.base_reference_,old);
  EXPECT_EQ(node.execution_revision_,before.execution_revision);
  const auto request=node.requested_reference_;
  const auto revision=node.requested_reference_revision_;
  node.receiveBaseReference(route(2.));
  EXPECT_EQ(node.requested_reference_,request);
  EXPECT_EQ(node.requested_reference_revision_,revision);
  EXPECT_TRUE(node.heartbeatSnapshotCurrent(before));
}

TEST_F(NodeReferenceContractTest, InitialReferenceIsInstalledWithoutAConnection) {
  stopBrainWorker();node.brain_mode_=true;
  node.receiveBaseReference(route(0.));
  ASSERT_TRUE(node.base_reference_);
  EXPECT_EQ(node.base_reference_,node.requested_reference_);
  EXPECT_EQ(node.reference_revision_,1U);
  EXPECT_FALSE(node.reference_transition_);
}

TEST_F(NodeReferenceContractTest, FreeRunAlongsideRetainsOrdinaryGeometryWithoutAdoptingOvertake) {
  auto command=prepare();other(0.,2.);
  node.receiveBaseReference(route(2.));
  auto snapshot=node.captureBrainInputs(true);
  ASSERT_TRUE(snapshot.alongside);ASSERT_TRUE(snapshot.ordinary_hold);
  EXPECT_FALSE(snapshot.active_command);
  EXPECT_FALSE(node.tryReferenceTransition(snapshot,command));
  const auto accept=[](const mppi::TemporaryReference &){return mppi::RejectReason::NONE;};
  auto hold=node.ordinaryExecutionFallback(snapshot,command,accept);
  ASSERT_GE(hold.trajectory.points.size(),3U);
  EXPECT_EQ(hold.mode,"FREE_RUN");
  for (const auto &p:hold.trajectory.points) EXPECT_NEAR(p.pose.position.y,0.,1e-9);
  node.publishOutput(hold);
  node.odometry_->pose.pose.position.y=.10;
  auto next=node.captureBrainInputs(true);
  EXPECT_EQ(next.ordinary_hold->generation,snapshot.ordinary_hold->generation);
  EXPECT_FALSE(node.active_brain_command_);
  EXPECT_EQ(node.driving_fsm_.mode(),DrivingMode::FREE_RUN);
}

TEST_F(NodeReferenceContractTest, EmptyV2xDoesNotReleaseAlongsideAndExpiredObservationIsUnknown) {
  auto command=prepare();other(0.,2.);
  node.receiveBaseReference(route(1.));
  ASSERT_TRUE(node.captureBrainInputs(true).alongside);
  node.receiveVehiclePositions(v2x_msgs::msg::V2XVehiclePositionArray{});
  EXPECT_TRUE(node.captureBrainInputs(true).alongside);
  node.observed_vehicles_.at("d3").stamp_sec=-1.;
  auto missing=node.captureBrainInputs(true);
  EXPECT_TRUE(missing.opponents.empty());
  EXPECT_TRUE(missing.alongside_unknown);
  EXPECT_FALSE(node.tryReferenceTransition(missing,command));
  other(-10.,2.);
  auto clear=node.captureBrainInputs(true);
  EXPECT_FALSE(clear.alongside_unknown);EXPECT_FALSE(clear.alongside);
  EXPECT_FALSE(clear.ordinary_hold);
}

TEST_F(NodeReferenceContractTest, ClearConnectionCommitsReferenceAndOutputTogetherWithoutChangingMode) {
  auto command=prepare();
  auto before=node.captureBrainInputs();
  node.receiveBaseReference(route(.5));
  auto snapshot=node.captureBrainInputs(true);
  ASSERT_TRUE(node.tryReferenceTransition(snapshot,command));
  EXPECT_EQ(node.base_reference_,node.requested_reference_);
  ASSERT_TRUE(node.reference_transition_);
  EXPECT_FALSE(node.active_brain_command_);
  EXPECT_EQ(node.driving_fsm_.mode(),DrivingMode::FREE_RUN);
  auto after=node.captureBrainInputs();
  ASSERT_TRUE(after.published_command);
  EXPECT_EQ(after.published_command->reason,"mppi_brain:reference_transition");
  EXPECT_FALSE(node.heartbeatSnapshotCurrent(before));
  node.receiveBaseReference(route(-.5));
  EXPECT_FALSE(node.tryReferenceTransition(node.captureBrainInputs(true),command));
  EXPECT_NE(node.base_reference_,node.requested_reference_);
}

TEST_F(NodeReferenceContractTest, ConnectionReleasesAfterProgressAndMeasuredAlignment) {
  auto command=prepare();
  node.receiveBaseReference(route(.5));
  ASSERT_TRUE(node.tryReferenceTransition(node.captureBrainInputs(true),command));
  node.odometry_->pose.pose.position.y=.5;
  node.odometry_->pose.pose.position.x=1.;
  EXPECT_TRUE(node.captureBrainInputs(true).reference_transition);
  node.odometry_->pose.pose.position.x=20.;
  auto completed=node.captureBrainInputs(true);
  EXPECT_FALSE(completed.reference_transition);
  EXPECT_FALSE(completed.ordinary_hold);
}

TEST_F(NodeReferenceContractTest, ChangedRequestCannotCommitAnEarlierValidatedConnection) {
  auto command=prepare();
  node.receiveBaseReference(route(.5));
  auto snapshot=node.captureBrainInputs();
  node.receiveBaseReference(route(1.));
  bool committed=false;
  EXPECT_FALSE(node.publishHeartbeatCommand(snapshot,command,[&]{committed=true;},true));
  EXPECT_FALSE(committed);
  EXPECT_NE(node.base_reference_,node.requested_reference_);
}

TEST_F(NodeReferenceContractTest, OccupiedConnectionCannotCommitByPublishingZeroSpeed) {
  auto command=prepare();node.receiveBaseReference(route(.5));
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>(*node.wall_map_);
  std::fill(map->data.begin(),map->data.end(),100);
  node.wall_map_=map;node.wall_index_=std::make_shared<OccupancyGridWallIndex>(map);
  EXPECT_FALSE(node.tryReferenceTransition(node.captureBrainInputs(true),command));
  EXPECT_NE(node.base_reference_,node.requested_reference_);
  EXPECT_FALSE(node.reference_transition_);
}

TEST_F(NodeReferenceContractTest, NewVehicleObservationInvalidatesAValidatedReferenceConnection) {
  auto command=prepare();node.receiveBaseReference(route(.5));
  auto snapshot=node.captureBrainInputs();
  v2x_msgs::msg::V2XVehiclePositionArray update;
  update.vehicles.resize(1);
  update.vehicles.front().vehicle_id="new_vehicle";
  update.vehicles.front().position.y=.5;
  node.receiveVehiclePositions(update);
  bool committed=false;
  EXPECT_FALSE(node.publishHeartbeatCommand(snapshot,command,[&]{committed=true;},true));
  EXPECT_FALSE(committed);
}

TEST_F(NodeReferenceContractTest, AdoptedOvertakeDefersReferenceRequests) {
  auto command=prepare();node.driving_fsm_.acceptLateral(DrivingMode::OVERTAKE);
  node.receiveBaseReference(route(.5));
  EXPECT_FALSE(node.tryReferenceTransition(node.captureBrainInputs(true),command));
  EXPECT_EQ(node.driving_fsm_.mode(),DrivingMode::OVERTAKE);
  EXPECT_NE(node.base_reference_,node.requested_reference_);
}

TEST_F(NodeReferenceContractTest, InvalidOrdinaryHoldRemainsAnExplicitInfeasibleBrakingOutput) {
  auto command=prepare();other(0.,2.);
  const auto snapshot=node.captureBrainInputs(true);
  const auto reject=[](const mppi::TemporaryReference &){return mppi::RejectReason::COLLISION;};
  auto output=node.ordinaryExecutionFallback(snapshot,command,reject);
  EXPECT_NE(output.reason.find("infeasible_braking_fallback"),std::string::npos);
  EXPECT_EQ(output.mode,"FREE_RUN");
  ASSERT_GE(output.trajectory.points.size(),3U);
  for (const auto &p:output.trajectory.points) {
    EXPECT_NEAR(p.pose.position.y,0.,1e-9);
    EXPECT_FLOAT_EQ(p.longitudinal_velocity_mps,0.);
  }
}

TEST_F(NodeReferenceContractTest, ExhaustedHoldUsesCommittedReferenceNotRequestedGeometry) {
  auto command=prepare();other(0.,2.);
  node.receiveBaseReference(route(4.));
  auto snapshot=node.captureBrainInputs(true);
  snapshot.ordinary_hold->trajectory.points.resize(2);
  const auto accept=[](const mppi::TemporaryReference &){return mppi::RejectReason::NONE;};
  auto output=node.ordinaryExecutionFallback(snapshot,command,accept);
  EXPECT_EQ(output.mode,"FREE_RUN");
  ASSERT_GE(output.trajectory.points.size(),3U);
  for (const auto &p:output.trajectory.points) EXPECT_NEAR(p.pose.position.y,0.,1e-8);
  EXPECT_NE(output.reason.find("ordinary_exhausted"),std::string::npos);
}
