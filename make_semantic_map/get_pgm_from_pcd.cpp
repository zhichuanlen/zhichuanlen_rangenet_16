#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>

std::string pcd_file = "/home/ubuntu/zhichuanlen_rangenet_16/make_semantic_map/test/final_points.pcd";
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

class point {
public:
	int r = 0;
	int g = 0;
    int b = 0;
};

std::vector<std::vector<point>> SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,  double min_z, double max_z)
{
    float map_resolution = 0.05;
    double x_min, x_max, y_min, y_max;
    std::vector<std::vector<point>> data;

    std::cout<<"cloud->points.size()  = "<<cloud->points.size()<<std::endl;
    for(int i = 0; i < cloud->points.size() - 1; i++)//求出地图的边界位置
    {
        if(i == 0)
        {
            x_min = x_max = cloud->points[i].x;
            y_min = y_max = cloud->points[i].y;
        }

        double x = cloud->points[i].x;
        double y = cloud->points[i].y;

        if(x < x_min) x_min = x;
        if(x > x_max) x_max = x;

        if(y < y_min) y_min = y;
        if(y > y_max) y_max = y;
    }

    int width = int((x_max - x_min) / map_resolution)+10;
    int height = int((y_max - y_min) / map_resolution)+10;
    std::cout<<"width  = "<< width <<std::endl;
    std::cout<<"height  = "<< height <<std::endl;
    
    data.resize(height);
    for(int i = 0 ; i < height ; i ++)   data[i].resize(width);

    for(int iter = 0; iter < cloud->points.size()-1; iter++)
    {
        int j = int((cloud->points[iter].x - x_min) / map_resolution);
        int i = int((cloud->points[iter].y - y_min) / map_resolution);

        data[i][j].r = cloud->points[iter].r;
        data[i][j].g = cloud->points[iter].g;
        data[i][j].b = cloud->points[iter].b;
    }
    return data;
}

void save_ppm(std::string &out_file,std::vector<std::vector<point>> data)
{
    int width = data[0].size();
    int height = data.size();
    FILE *fp;
   	int i, j;
	char chHeader[100] = {0};
    unsigned char *canvas = (unsigned char *)malloc(sizeof(unsigned char) * width * height * 3);
 
	//初始化画布
    memset(canvas, 255, width * height * 3);
	//打开文件
	fp = fopen(out_file.c_str(), "w");
	if(NULL == fp){
		return;
	}
	//写入ppm文件头
	sprintf(chHeader, "P6 %d %d 255 ", width, height);
	fwrite(chHeader, sizeof(unsigned char), strlen(chHeader), fp);
	//写入图像数据
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            	int index = i * width * 3 + j * 3;
                int r = data[i][j].r;
                int g = data[i][j].g;
                int b = data[i][j].b;
            	canvas[index] = r; // Red
            	canvas[index + 1] = g; // Green
            	canvas[index + 2] = b; // Blue
        }
    }
	//保存图像数据到文件
    fwrite(canvas, sizeof(unsigned char), width * height * 3, fp);
 
	fclose(fp);
	free(canvas);
	return;
}


int main(int argc, char** argv)
{
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcd_file, *pcd_cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file: %s \n", pcd_file.c_str());
        return (-1);
    }

    std::cout << "初始点云数据点数：" << pcd_cloud->points.size() << std::endl;

    std::string out_file = "out1.ppm";
    // std::vector<std::vector<point>> vector_points = SetMapTopicMsg(pcd_cloud,  -5.0, 5.0);
    save_ppm(out_file,SetMapTopicMsg(pcd_cloud,  -5.0, 5.0));


   return 0;
}