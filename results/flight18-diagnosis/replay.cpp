#define main unused_live_main
#include "../../src/openvins_runner/vio_live.cpp"
#undef main

class DiagnosticVio : public LoggedVioManager {
public:
 using LoggedVioManager::LoggedVioManager;
 bool zupt() { return did_zupt_update; }
};
struct Sample { int64_t t; Eigen::Vector3d v; };
std::vector<Sample> samples(std::string path, double scale, ImuLowPass &filter) {
 std::ifstream in(path, std::ios::binary); std::vector<Sample> out;
 ImuLowPassState fs; char r[16];
 while(in.read(r,16)) { int64_t t; int16_t xyz[3]; memcpy(&t,r+8,8);memcpy(xyz,r,6);
 Eigen::Vector3d v(xyz[0]*scale,xyz[1]*scale,xyz[2]*scale);
 out.push_back({t-filter.delay_ns,fs.step(filter,t,v)}); }
 return out;
}
int main(int argc,char **argv) {
 if(argc!=5) return 2;
 std::string prefix=argv[1],config=argv[2],output=argv[3],variant=argv[4];
 auto parser=std::make_shared<ov_core::YamlParser>(config);
 ov_core::Printer::setPrintLevel("DEBUG");
 ov_msckf::VioManagerOptions params; params.print_and_load(parser);
 params.use_multi_threading_subs=false;
 if(variant=="beginning") params.zupt_only_at_beginning=true;
 if(variant=="no-disparity") params.zupt_max_disparity=0;
 if(variant=="gates5") {params.msckf_options.chi2_multipler=5;params.slam_options.chi2_multipler=5;}
 if(!parser->successful()) return 3;
 auto sys=std::make_shared<DiagnosticVio>(params);
 ImuLowPass filter;filter.design(50,440);
 auto acc=samples(prefix+".imu_accel.bin",0.004785645,filter);
 auto gyr=samples(prefix+".imu_gyro.bin",0.001221729,filter);
 std::vector<ov_core::ImuData> imu;size_t ai=0;
 for(auto &g:gyr) {
  if(g.t<acc.front().t || g.t>acc.back().t)continue;
  while(ai+1<acc.size() && acc[ai+1].t<=g.t)ai++;
  Eigen::Vector3d av=acc[ai].v;
  if(ai+1<acc.size())av+=double(g.t-acc[ai].t)/double(acc[ai+1].t-acc[ai].t)*(acc[ai+1].v-acc[ai].v);
  ov_core::ImuData im;im.timestamp=g.t*1e-9;im.am=av;im.wm=g.v;imu.push_back(im);
 }
 std::ifstream ts(prefix+".timestamps.txt"),raw(prefix+".y16",std::ios::binary);
 std::ofstream out(output);out.precision(12);
 out<<"# time zupt px py pz vx vy vz bax bay baz dt clones\n";
 int64_t ns;size_t ii=0,frame=0;std::vector<uint8_t> bytes(1280*800*2);
 while(ts>>ns && raw.read((char*)bytes.data(),bytes.size())) {
  double t=ns*1e-9;
  double dt=sys->get_state()->_calib_dt_CAMtoIMU->value()(0);
  while(ii<imu.size() && (ii==0 || imu[ii-1].timestamp<=t+dt))sys->feed_measurement_imu(imu[ii++]);
  cv::Mat im(800,1280,CV_8UC1);
  for(size_t j=0;j<1280*800;j++)im.data[j]=bytes[2*j+1];
  ov_core::CameraData cam;cam.timestamp=t;cam.sensor_ids={0};cam.images={im};cam.masks={cv::Mat::zeros(800,1280,CV_8UC1)};
  printf("DIAG_FRAME %zu %.9f\n",frame++,t);sys->feed_measurement_camera(cam);
  if(sys->initialized_time()>0) {
   auto s=sys->get_state();auto imust=s->_imu;
   out<<s->_timestamp<<' '<<sys->zupt()<<' '<<imust->pos().transpose()<<' '<<imust->vel().transpose()<<' '<<imust->bias_a().transpose()<<' '<<s->_calib_dt_CAMtoIMU->value()(0)<<' '<<s->_clones_IMU.size()<<'\n';
  }
 }
}
