#include <omp.h>
#include <ctime>
#include <vector>
#include <string>
#include <dirent.h>
#include <algorithm>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/boost.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>  // pcl::transformPointCloud
#include <pcl/visualization/pcl_visualizer.h>  // pcl::visualization::PCLVisualizer
#include <pcl/filters/voxel_grid.h>  //体素滤波器头文件
#include <sys/stat.h>
#include <boost/program_options.hpp>



//namespace fs = std::experimental::filesystem::v1;

static std::vector<std::string> file_lists;
static std::vector<std::string> label_lists;

 std::map<int,std::vector<int>> dict_color = {
                                { 0, {0, 0, 0} },
                                {  1 , {0, 0, 255} },
                                { 10 , {245, 150, 100} },
                                {11, {245, 230, 100} },
                                {13, {250, 80, 100} },
                                {15, {150, 60, 30} },
                                {16, {255, 0, 0} },
                                {18, {180, 30, 80} },
                                {20, {255, 0, 0} },
                                {30, {30, 30, 255} },
                                {31, {200, 40, 255} },
                                {32, {90, 30, 150} },
                                {40, {255, 0, 255} },
                                {44, {255, 150, 255} },
                                {48, {75, 0, 75} },
                                {49, {75, 0, 175} },
                                {50, {110, 200, 255} },
                                {51, {50, 120, 255} },
                                {52, {0, 150, 255} },
                                {60, {170, 255, 150} },
                                {70, {0, 175, 0} },
                                {71, {0, 60, 135} },
                                {72, {80, 240, 150} },
                                {80, {150, 240, 255} },
                                {81, {0, 0, 255} },
                                {99, {255, 255, 50} },
                                {252, {245, 150, 100} },
                                {256, {255, 0, 0} },
                                {253, {200, 40, 255} },
                                {254, {30, 30, 255} },
                                {255, {90, 30, 150} },
                                {257, {250, 80, 100} },
                                {258, {180, 30, 80} },
                                {259, {255, 0, 0} },
                                {299, {255,255,255} }
               };
                                

class CommandLineArgs
{
public:
    CommandLineArgs(int argc, char **argv);
    ~CommandLineArgs(){}
    bool process_command_line(
        int argc, 
        char** argv);

    std::string _bin_path;
    std::string _pcd_path;
private:
    CommandLineArgs(){}
    boost::program_options::options_description *_desc;  


};

CommandLineArgs::CommandLineArgs(int argc, char **argv)
{
    _bin_path = "/home/kitti_velodyne_bin_to_pcd/bin/";
    _pcd_path = "/home/kitti_velodyne_bin_to_pcd/pcd/";

    _desc = new boost::program_options::options_description("Program Usage", 1024, 512);
    _desc->add_options()
                    ("help",     "produce help message")
                    ("b",   boost::program_options::value<std::string>(&_bin_path)->required(), "bin file folder")
                    ("p",   boost::program_options::value<std::string>(&_pcd_path)->required(), "pcd file folder")            
                    ;
    process_command_line(argc, argv);
}

bool CommandLineArgs::process_command_line(
        int argc, 
        char** argv)
{
    try
    {

        boost::program_options::variables_map vm;
        boost::program_options::store(boost::program_options::parse_command_line(argc, argv, *_desc), vm);

        if (vm.count("help"))
        {
            std::cout << *_desc << "\n";
            return false;
        }

        // There must be an easy way to handle the relationship between the
        // option "help" and "host"-"port"-"config"
        // Yes, the magic is putting the po::notify after "help" option check
        boost::program_options::notify(vm);
    }
    catch (std::exception &e)
    {
        std::cerr << "Error: " << e.what() << "\n";
        return false;
    }
    catch (...)
    {
        std::cerr << "Unknown error!"
                  << "\n";
        return false;
    }
    return true;
}

void read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type)
{
    struct dirent *ptr;
    DIR *dir;
    //dir = opendir(dir_path.c_str());
    if (!(dir = opendir(dir_path.c_str())))
    {
        std::cout<<"cant opendir"<<std::endl;
        return;
    }
    out_filelsits.clear();
    while ((ptr = readdir(dir)) != NULL){
        std::string tmp_file = ptr->d_name;
        if (tmp_file[0] == '.')continue;
        if (type.size() <= 0){
            out_filelsits.push_back(ptr->d_name);
        }else{
            if (tmp_file.size() < type.size())continue;
            std::string tmp_cut_type = tmp_file.substr(tmp_file.size() - type.size(),type.size());
            if (tmp_cut_type == type){
                out_filelsits.push_back(ptr->d_name);
            }
        }
    }
}

bool computePairNum(std::string pair1,std::string pair2)
{
    return pair1 < pair2;
}

void sort_filelists(std::vector<std::string>& filists,std::string type)
{
    if (filists.empty())return;

    std::sort(filists.begin(),filists.end(),computePairNum);
}


std::vector<std::string> split(const std::string& line, const std::string& delim, bool skipEmpty = false)
{
  std::vector<std::string> tokens;

  boost::char_separator<char> sep(delim.c_str(), "", (skipEmpty ? boost::drop_empty_tokens : boost::keep_empty_tokens));
  boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);

  for (auto it = tokenizer.begin(); it != tokenizer.end(); ++it)
    tokens.push_back(*it);

  return tokens;
}

std::string trim(const std::string& str, const std::string& whitespaces = " \0\t\n\r\x0B" )
{
  int32_t beg = 0;
  int32_t end = 0;

  /** find the beginning **/
  for (beg = 0; beg < (int32_t) str.size(); ++beg)
  {
    bool found = false;
    for (uint32_t i = 0; i < whitespaces.size(); ++i)
    {
      if (str[beg] == whitespaces[i])
      {
        found = true;
        break;
      }
    }
    if (!found) break;
  }

  /** find the end **/
  for (end = int32_t(str.size()) - 1; end > beg; --end)
  {
    bool found = false;
    for (uint32_t i = 0; i < whitespaces.size(); ++i)
    {
      if (str[end] == whitespaces[i])
      {
        found = true;
        break;
      }
    }
    if (!found) break;
  }

  return str.substr(beg, end - beg + 1);
}

/*               #################################################################################         */


pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_in)
{
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    sor.setInputCloud(points_in);             //输入点云
    sor.setLeafSize(0.5f, 0.5f, 0.5f); //体素滤波器，单位m
    sor.filter(*points_filtered);          //滤波后的点云
    return points_filtered;
}



std::vector<Eigen::Matrix4f> loadPoses(const std::string& file_name) {
  std::vector<Eigen::Matrix4f> poses;
  std::ifstream fp(file_name.c_str());
  std::string line;

  if (!fp.is_open()) return poses;
  fp.peek();

  while (fp.good()) {
    Eigen::Matrix4f P = Eigen::Matrix4f::Identity();

    std::getline(fp, line);
    std::vector<std::string> entries = split(line, " ");

    if (entries.size() < 12) {
      fp.peek();
      continue;
    }

    for (uint32_t i = 0; i < 12; ++i) {
      P(i / 4, i - int(i / 4) * 4) = boost::lexical_cast<float>(trim(entries[i]));
    }

    poses.push_back(P);

    fp.peek();
  }

  fp.close();

  return poses;
}


std::vector<Eigen::Matrix4f> get_groundtruth(const std::string& file_name)
{
    std::vector<Eigen::Matrix4f> poses =  loadPoses(file_name);
    std::vector<Eigen::Matrix4f> groundtruth;
    Eigen::Matrix4f Tr ;
    Tr<< 4.276802385584e-04,-9.999672484946e-01,-8.084491683471e-03,-1.198459927713e-02,-7.210626507497e-03 ,8.081198471645e-03 ,-9.999413164504e-01, -5.403984729748e-02, 9.999738645903e-01 ,4.859485810390e-04, -7.206933692422e-03, -2.921968648686e-01,0,0,0,1;
    Eigen::Matrix4f Tr_inv = Tr.inverse();

    for (uint32_t i = 0; i < poses.size(); ++i) {
        Eigen::Matrix4f pose = poses[i];
        groundtruth.push_back(Tr_inv * pose * Tr);
    }
    return groundtruth;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr load_a_semantic_points(std::string &in_file, std::string &in_file_label)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_label (new pcl::PointCloud<pcl::PointXYZRGB>);

    // load point cloud and lables
    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    std::fstream input_label(in_file_label.c_str(), std::ios::in | std::ios::binary);
    if(!input.good()){
        std::cerr << "Could not read file: " << in_file << std::endl;
        exit(EXIT_FAILURE);
    }
    if(!input_label.good()){
        std::cerr << "Could not read file: " << in_file_label << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);
    input_label.seekg(0, std::ios::beg);

    int i;
    for (int i=0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;
        pcl::PointXYZRGB point_label;
        unsigned int label_;
        input_label.read((char *) &label_, sizeof(unsigned int));
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));

        point_label.x = point.x;
        point_label.y = point.y;
        point_label.z = point.z;

        if (dict_color.count(label_) != 0)
        {
            point_label.r = dict_color[label_][0];
            point_label.g = dict_color[label_][1];
            point_label.b = dict_color[label_][2];
        }
        else 
        {
            point_label.r = 100;
            point_label.g = 100;
            point_label.b = 100;
        }  
        points->push_back(point);
        points_label->push_back(point_label);
        
    }
    std::cout<<std::endl;
    input.close();
    input_label.close();

    return points_label;
}



void save_semantic_pcd_from_KittiPclBinData(std::string &in_file, std::string &in_file_label,std::string& out_file)
{
    // load point cloud
    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if(!input.good()){
        std::cerr << "Could not read file: " << in_file << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    // load  labels
    std::fstream input_label(in_file_label.c_str(), std::ios::in | std::ios::binary);
    if(!input_label.good()){
        std::cerr << "Could not read file: " << in_file_label << std::endl;
        exit(EXIT_FAILURE);
    }
    input_label.seekg(0, std::ios::beg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_label (new pcl::PointCloud<pcl::PointXYZRGB>);


    int i;
    for (int i=0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;
        pcl::PointXYZRGB point_label;
        unsigned int label_;
        input_label.read((char *) &label_, sizeof(unsigned int));
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));

        point_label.x = point.x;
        point_label.y = point.y;
        point_label.z = point.z;

        if (dict_color.count(label_) != 0)
        {
            point_label.r = dict_color[label_][0];
            point_label.g = dict_color[label_][1];
            point_label.b = dict_color[label_][2];
        }
        else 
        {
            point_label.r = 100;
            point_label.g = 100;
            point_label.b = 100;
        }  
        points->push_back(point);
        points_label->push_back(point_label);
        
    }


    std::cout<<std::endl;
    input.close();
    input_label.close();

    std::cout << "Read KTTI point cloud with " << i << " points, writing to " << out_file << std::endl;
    pcl::PCDWriter writer;
    pcl::PCDWriter writer_label;

    // Save DoN features
    // writer.write< pcl::PointXYZI > (out_file, *points, true);
    writer_label.write< pcl::PointXYZRGB > (out_file, *points_label, true);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr semantic_points,Eigen::Matrix4f tfMatrix)
{
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*semantic_points, *transformed_cloud, tfMatrix);
    return transformed_cloud;
}

void save_semantic_pcd(std::string &in_file,pcl::PointCloud<pcl::PointXYZRGB>::Ptr points)
{
    pcl::PCDWriter writer_label;
    writer_label.write< pcl::PointXYZRGB > (in_file, *points, true);
}


int main(int argc, char **argv)
{
    
    CommandLineArgs cmd_args(argc, argv);

    // Create _outputFile folder if not exist
    struct stat sb;
    std::string folderPath = cmd_args._pcd_path;
    if (! (stat(folderPath.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)) )
    {//...It is not a directory...
        mkdir(folderPath.c_str(), 0755);
    }
    folderPath = cmd_args._bin_path;
    if (! (stat(folderPath.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)) )
    {//...It is not a directory...
        mkdir(folderPath.c_str(), 0755);
    }

    
    std::cout<<"bin2pcd"<<std::endl;
    read_filelists( cmd_args._bin_path, file_lists, "bin" );
    sort_filelists( file_lists, "bin" );

    read_filelists( cmd_args._bin_path+ "../labels/", label_lists, "label" );
    sort_filelists( label_lists, "label" );

    #pragma omp parallel num_threads(8)
    #pragma omp parallel for


    std::string pose_file  = cmd_args._bin_path+ "../poses.txt" ;
    std::vector<Eigen::Matrix4f> groundtruth = get_groundtruth(pose_file);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_points(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (int i = 0; i < file_lists.size(); ++i)
    // for (int i = 0; i < 1; ++i)
    {
        if (i<file_lists.size()-1) printf("\r读取中[%.2lf%%]", i*100.0 / (file_lists.size() - 1));
		else printf("\r读取完成[%.2lf%%]", i*100.0 / (file_lists.size() - 1));

        std::string bin_file = cmd_args._bin_path + file_lists[i];
        std::string label_file = cmd_args._bin_path+ "../labels/"  + label_lists[i];
        std::string tmp_str = file_lists[i].substr(0, file_lists[i].length() - 4) + ".pcd";
        std::string pcd_file = cmd_args._pcd_path + tmp_str;
        // save_semantic_pcd_from_KittiPclBinData( bin_file, label_file,pcd_file );
        
        //加载.bin和.label合成一个XYZRGB的pcl点云
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr semantic_points = load_a_semantic_points(bin_file, label_file);
        //将点云做刚体坐标变换
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_points = transform_points(semantic_points, groundtruth[i]);

        *final_points = *final_points + *transformed_points;
        final_points = voxel_filter(final_points);

        //保存点云为pcd格式
        // save_semantic_pcd(pcd_file, transformed_points);
    }
    std::string final_pcd_file = cmd_args._pcd_path+"/final_points.pcd";
    save_semantic_pcd(final_pcd_file, final_points);
    return 0;
}

