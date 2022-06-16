#include "Utils.h"
#include <bits/stdc++.h>
#include <boost/algorithm/string.hpp>
#include "Bundler.h"
#include "DataLoader.h"


int main(int argc, char **argv)
{
  std::shared_ptr<YAML::Node> yml(new YAML::Node);
  if (argc<2)
  {
    printf("Please provide path to config file\n");
    exit(1);
  }

  std::string config_dir = std::string(argv[1]);
  *yml = YAML::LoadFile(config_dir);

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  DataLoaderYcbineoat data_loader(yml);

  const std::string base_dir = (*yml)["debug_dir"].as<std::string>();
  std::string cmd = "rm -rf "+base_dir+" && mkdir -p "+base_dir+" && mkdir -p "+base_dir+"/color_viz/";
  system(cmd.c_str());


  Eigen::Matrix4f ob_in_cam_last(Eigen::Matrix4f::Identity());

  Bundler bundler(yml,&data_loader);

  while (data_loader.hasNext())
  {
    std::shared_ptr<Frame> frame = data_loader.next();
    if (!frame) break;
    const std::string index_str = frame->_id_str;
    const std::string out_dir = (*yml)["debug_dir"].as<std::string>()+"/"+index_str+"/";
    cv::imwrite(out_dir+index_str+"_color.png",frame->_color);

    Eigen::Matrix4f cur_in_model(data_loader._ob_in_cam0.inverse());
    bundler.processNewFrame(frame);

    bundler.saveNewframeResult();

  }
}