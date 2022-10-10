#include "../include/OccupancyGrid.h"

OccupancyGrid::OccupancyGrid(size_t w, size_t h, double resolution)
{
    occupancy_grid.header.frame_id = "map";
    occupancy_grid.info.width = w;
    occupancy_grid.info.height = h;
    occupancy_grid.info.resolution = resolution;
    occupancy_grid.info.origin.position.x = -(w*resolution)/2;
    occupancy_grid.info.origin.position.y = -(h*resolution)/2;
    occupancy_grid.info.origin.position.z = 0.0;
    occupancy_grid.data.resize(w*h);
    image = cv::Mat(w,h, CV_8U,50);

    free = cv::Mat::zeros(w,h,CV_16U);
    occupied = cv::Mat::zeros(w,h,CV_16U);
    prob = cv::Mat::zeros(w,h,CV_16U);

}

void OccupancyGrid::resize_image(size_t up, size_t down, size_t left, size_t right)
{
    printf("Padding image by r/u/l/d: %zu, %zu, %zu, %zu\n",right,up,left,down);
    //cv image starts from bottom left corner, thus expansion direction in up/dpown is flipped
    cv::copyMakeBorder(image, image, down, up, left, right,cv::BORDER_CONSTANT, 0);
    cv::copyMakeBorder(free, free, down, up, left, right,cv::BORDER_CONSTANT, 0);
    cv::copyMakeBorder(occupied, occupied, down, up, left, right,cv::BORDER_CONSTANT, 0);
    cv::copyMakeBorder(prob, prob, down, up, left, right,cv::BORDER_CONSTANT, 0);

    occupancy_grid.info.width = image.cols;
    occupancy_grid.info.height = image.rows;
    occupancy_grid.info.origin.position.y -= ((down) * occupancy_grid.info.resolution);
    occupancy_grid.info.origin.position.x -= ((left) * occupancy_grid.info.resolution);
    occupancy_grid.info.origin.position.z = 0.0;

}

void OccupancyGrid::clear()
{
    image = image.zeros(image.size(), image.type());
    free = free.zeros(free.size(),free.type());
    occupied = occupied.zeros(occupied.size(),occupied.type());
    prob = prob.zeros(prob.size(),prob.type());
}

bool OccupancyGrid::set(size_t x, size_t y, int value)
{
    if(x >= occupancy_grid.info.width|| y >= occupancy_grid.info.height)return false;
    size_t index = (y*occupancy_grid.info.width) + x;
    occupancy_grid.data.at(index) = value;
    return true;
}

void OccupancyGrid::line(size_t x, size_t y, size_t x2, size_t y2, int value)
{
    for( auto p : bresenham_line(x,y,x2,y2)) set(p.x,p.y,value);
}

void OccupancyGrid::cv_line(size_t x, size_t y, size_t x2, size_t y2, int value)
{
    cv::line(image,cv::Point(x,y),cv::Point(x2,y2),cv::Scalar(value),10,cv::LINE_4,0);
}

bool OccupancyGrid::cv_mark_occupied(size_t x, size_t y,uint16_t value)
{
    if(x>=occupied.cols||y>=occupied.rows)return false;
    occupied.at<u_int16_t>(y,x)+=value;
    return true;
}

bool OccupancyGrid::cv_mark_free(size_t x, size_t y,uint16_t value)
{
    if(x>=free.cols||y>=free.rows)return false; 
    free.at<u_int16_t>(y,x)+=value;
    return true;
}

void OccupancyGrid::cvMat_to_occupancy_grid()
{
    printf("image size is %d, %d\n",image.cols, image.rows);
    occupancy_grid.data = std::vector<signed char>(image.datastart, image.dataend);
}

void OccupancyGrid::blur_counters(int ksize)
{
    cv::GaussianBlur(occupied,occupied,cv::Size(ksize,ksize),0);
    cv::GaussianBlur(free,free,cv::Size(ksize,ksize),0);
}

void OccupancyGrid::process_free_and_occupied(uint16_t free_thresh=5, int ksize=3)
{
    cv::Mat occupied_temp, free_temp;
    cv::GaussianBlur(occupied, occupied_temp,cv::Size(ksize,ksize),0);
    cv::GaussianBlur(free,free_temp,cv::Size(ksize,ksize),0);

    for(int i=0; i!=image.rows;i++)
    {
        for(int j=0; j!=image.cols;j++)
        {
            float prob_free = 1.0 - float(occupied_temp.at<uint16_t>(i,j)) / float(free_temp.at<uint16_t>(i,j));
            if(prob_free>0.55 && free_temp.at<uint16_t>(i,j)>free_thresh)image.at<int8_t>(i,j) = 0;
            else if(prob_free<0.5)image.at<int8_t>(i,j) = 100;
            else image.at<int8_t>(i,j) = 50;
        }
    }

    size_t x, y;
    get_map_origin(x,y);
    cv::circle(image,{x, y},10,{100,0,0},-1);

}