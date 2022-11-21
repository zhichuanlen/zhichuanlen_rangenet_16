#include <omp.h>
#include <ctime>
#include <vector>
#include <string>
#include <dirent.h>
#include <algorithm>
#include <typeinfo>
#include <iostream>
#include <sys/stat.h>

#include <boost/program_options.hpp>

#include <fstream>
#include <cmath>


static std::vector<std::string> file_lists;
static std::vector<std::string> label_lists;

char k;

class CommandLineArgs
{
public:
    CommandLineArgs(int argc, char **argv);
    ~CommandLineArgs(){}
    bool process_command_line(
        int argc, 
        char** argv);

    std::string _bin_path;
    std::string _label_path;
    std::string _bin_out_path;
    std::string _label_out_path;

    std::string _mode;
private:
    CommandLineArgs(){}
    boost::program_options::options_description *_desc;  


};

CommandLineArgs::CommandLineArgs(int argc, char **argv)
{
    _bin_path = "/media/ubuntu/zhi_chuan_len-/lidar_data/test/bin_test/";
    _label_path = "/media/ubuntu/zhi_chuan_len-/lidar_data/test/label_test/";

    _bin_out_path = "/media/ubuntu/zhi_chuan_len-/lidar_data/test/bin_out_test/";
    _label_out_path = "/media/ubuntu/zhi_chuan_len-/lidar_data/test/label_out_test/";


    _desc = new boost::program_options::options_description("Program Usage", 1024, 512);
    _desc->add_options()
                    ("help",     "produce help message")
                    ("b",   boost::program_options::value<std::string>(&_bin_path)->required(), "bin input file folder")
                    ("l",   boost::program_options::value<std::string>(&_label_path)->required(), "label input file folder")
                    ("bo",   boost::program_options::value<std::string>(&_bin_out_path)->required(), "bin file output folder")
                    ("lo",   boost::program_options::value<std::string>(&_label_out_path)->required(), "label file output folder")
                    ("m",   boost::program_options::value<std::string>(&_mode)->required(),     "mode -  make_dataset  make_dataset_bin_only ")                    
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



void make_dataset(std::string &in_bin_file, std::string &in_label_file , std::string &out_bin_file ,std::string &out_label_file)
{
    // load file
    std::ifstream input_bin(in_bin_file.c_str(), std::ios::in | std::ios::binary);
    std::ifstream input_label(in_label_file.c_str(), std::ios::in | std::ios::binary);
    std::ofstream my_bin_File (out_bin_file.c_str(), std::ios::out | std::ios::binary);
    std::ofstream my_label_File (out_label_file.c_str(), std::ios::out | std::ios::binary);

    input_bin.seekg(0, std::ios::beg);
    input_label.seekg(0, std::ios::beg);

    int i;
    int count = 0;
    for (i=0; input_bin.good() && !input_bin.eof(); ++i) {
        float l;
        float p_x,p_y,p_z,p_i;
        //read bin&label
        input_bin.read((char *) &p_x, sizeof(uint32_t));
        input_bin.read((char *) &p_y, sizeof(uint32_t));
        input_bin.read((char *) &p_z, sizeof(uint32_t));
        input_bin.read((char *) &p_i, sizeof(uint32_t));
        input_label.read((char*)&l ,  sizeof(uint32_t));


        float r = 0;
        r = sqrt(p_x*p_x+p_y*p_y);
        int whitch_laser = abs((int)(atan2(p_z,r)*180/(3.1416*0.4)-6));       //计算这一个点属于第几线激光,0.4是垂直分辨率
        //read bin&label
        if(whitch_laser%4==0 && i%2==0)
        {
            my_bin_File.write((char*)&p_x, sizeof(float));
            my_bin_File.write((char*)&p_y, sizeof(float));
            my_bin_File.write((char*)&p_z, sizeof(float));
            my_bin_File.write((char*)&p_i, sizeof(float));

            my_label_File.write((char*)&l, sizeof(float));
            count++;
        }

    }
    input_bin.close();
    input_label.close();

    my_bin_File.close();
    my_label_File.close();

    std::cout<<"read points= :"<< i <<std::endl;
    std::cout<<"read labels= :"<< i <<std::endl;

    
}



void make_dataset_bin_only(std::string &in_bin_file , std::string &out_bin_file)
{
    // load file
    std::ifstream input_bin(in_bin_file.c_str(), std::ios::in | std::ios::binary);
    std::ofstream my_bin_File (out_bin_file.c_str(), std::ios::out | std::ios::binary);

    input_bin.seekg(0, std::ios::beg);

    int i;
    int count = 0;
    for (i=0; input_bin.good() && !input_bin.eof(); ++i) {


        float p_x,p_y,p_z,p_i;
		//read bin
        input_bin.read((char *) &p_x, sizeof(uint32_t));
        input_bin.read((char *) &p_y, sizeof(uint32_t));
        input_bin.read((char *) &p_z, sizeof(uint32_t));
        input_bin.read((char *) &p_i, sizeof(uint32_t));
        

        float r = 0;
        r = sqrt(p_x*p_x+p_y*p_y);
        int whitch_laser = abs((int)(atan2(p_z,r)*180/(3.1416*0.4)-6));
        //write bin
        if(whitch_laser%4==0 && i%2==0)
        {
            my_bin_File.write((char*)&p_x, sizeof(float));
            my_bin_File.write((char*)&p_y, sizeof(float));
            my_bin_File.write((char*)&p_z, sizeof(float));
            my_bin_File.write((char*)&p_i, sizeof(float));

            count++;
        }

    }
    input_bin.close();
    std::cout<<std::endl;
    my_bin_File.close();

    std::cout<<"read points= :"<< i <<std::endl;
    std::cout<<"write points= :"<< count <<std::endl;
}


int main(int argc, char **argv)
{
    
    CommandLineArgs cmd_args(argc, argv);

    // 如果输出文件夹不存在就创建一个
    struct stat sb;
    std::string folderPath = cmd_args._bin_out_path;
    if (! (stat(folderPath.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)) )
    {//...It is not a directory...
        mkdir(folderPath.c_str(), 0755);
    }
    folderPath = cmd_args._label_out_path;
    if (! (stat(folderPath.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)) )
    {//...It is not a directory...
        mkdir(folderPath.c_str(), 0755);
    }



    if(cmd_args._mode == "make_dataset")
    {
        std::cout<<"make_dataset"<<std::endl;
        read_filelists( cmd_args._bin_path, file_lists, "bin" );
        sort_filelists( file_lists, "bin" );
        std::cout<<"num of bin file:"<<file_lists.size()<<std::endl;

        read_filelists( cmd_args._label_path, label_lists, "label" );
        sort_filelists( label_lists, "label" );
        std::cout<<"num of label file:"<<label_lists.size()<<std::endl;

        std::cout << "Run make_dataset" << std::endl;
        #pragma omp parallel num_threads(8)
        #pragma omp parallel for
        for (int i = 0; i < file_lists.size(); ++i)
        {
            std::string bin_file = cmd_args._bin_path + file_lists[i];
            std::string label_file = cmd_args._label_path + label_lists[i];

            std::string bin_out_file = cmd_args._bin_out_path + file_lists[i];
            std::string label_out_file = cmd_args._label_out_path + label_lists[i];

            std::cout << bin_file << "\n"
                      << bin_out_file << "\n"
                      << label_file << "\n"
                      << label_out_file << std::endl;
            make_dataset( bin_file, label_file , bin_out_file , label_out_file);
        }

    }
    else if(cmd_args._mode == "make_dataset_bin_only")
    {
        std::cout<<"make_dataset_bin_only"<<std::endl;
        read_filelists( cmd_args._bin_path, file_lists, "bin" );
        sort_filelists( file_lists, "bin" );
        std::cout<<"num of bin file:"<<file_lists.size()<<std::endl;

        std::cout << "Run make_dataset" << std::endl;
        #pragma omp parallel num_threads(8)
        #pragma omp parallel for
        for (int i = 0; i < file_lists.size(); ++i)
        {
            std::string bin_file = cmd_args._bin_path + file_lists[i];

            std::string bin_out_file = cmd_args._bin_out_path + file_lists[i];

            std::cout << bin_file << "\n"
                      << bin_out_file << std::endl;
            make_dataset_bin_only( bin_file,  bin_out_file);
        }

    }
    else
    {
        std::cout << "No mode provided" << std::endl;
    }
    

    

    return 0;
}