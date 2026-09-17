#include "reference_space_mppi_planner/reference_geometry.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
using namespace reference_space_mppi_planner;
void require(bool value) { if (!value) throw std::runtime_error("reference geometry regression"); }
int main(int argc, char **argv) {
  require(argc == 2);
  std::ifstream file(argv[1]); require(file.good());
  std::vector<ReferenceXY> raw;
  for (std::string line; std::getline(file,line);) {
    std::istringstream in(line); double x,y; char comma; in>>x>>comma>>y;
    require(!in.fail());raw.push_back({x,y});
  }
  const auto p=smoothReferenceGeometry(raw,false,2.0);
  require(p.size()==raw.size());
  double min_forward=1., max_curvature=0.;
  std::vector<ReferenceXY> shifted;
  for (std::size_t i=1;i+1<p.size();++i) {
    const double yaw=std::atan2(p[i+1][1]-p[i-1][1],p[i+1][0]-p[i-1][0]);
    shifted.push_back({p[i][0]+.7*std::sin(yaw),p[i][1]-.7*std::cos(yaw)});
  }
  for (std::size_t i=10;i+11<shifted.size();++i) {
    const auto &a=shifted[i-1], &b=shifted[i], &c=shifted[i+1];
    const double tx=p[i+2][0]-p[i+1][0],ty=p[i+2][1]-p[i+1][1];
    const double ux=b[0]-a[0],uy=b[1]-a[1],vx=c[0]-b[0],vy=c[1]-b[1];
    min_forward=std::min(min_forward,(vx*tx+vy*ty)/std::hypot(tx,ty));
    max_curvature=std::max(max_curvature,std::abs(2*(ux*vy-uy*vx)/
        (std::hypot(ux,uy)*std::hypot(vx,vy)*std::hypot(c[0]-a[0],c[1]-a[1]))));
  }
  std::cout<<"min_forward="<<min_forward<<" max_curvature="<<max_curvature<<std::endl;
  // This fixture checks the discretization reversal. Vehicle feasibility is
  // evaluated by the delayed rollout, not by assigning a steering guarantee
  // to every point of the underlying course's offset curve.
  require(min_forward>0.);require(max_curvature<1.);
  std::vector<ReferenceXY> line;
  for(int i=0;i<80;++i)line.push_back({i*.17,2.*i*.17});
  const auto straight=smoothReferenceGeometry(line,false,2.);
  for(std::size_t i=0;i<line.size();++i)
    require(std::hypot(straight[i][0]-line[i][0],straight[i][1]-line[i][1])<1e-10);
  std::vector<ReferenceXY> circle;
  for(int i=0;i<=200;++i)circle.push_back({20*std::cos(i*2*M_PI/200),20*std::sin(i*2*M_PI/200)});
  const auto ring=smoothReferenceGeometry(circle,true,2.);
  require(std::hypot(ring.front()[0]-ring.back()[0],ring.front()[1]-ring.back()[1])<1e-10);
  for(std::size_t i=0;i<ring.size();++i)require(std::hypot(ring[i][0],ring[i][1])>19.8);
  std::cout<<"Recorded offset corner, straight invariance and closed seam passed\n";
}
