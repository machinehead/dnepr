# include "kinect_utils/resamplePointCloud.h"

using namespace std;

//===============================================================================================
void resample(const sensor_msgs::PointCloud2 &inputPC, sensor_msgs::PointCloud2& outputPC,  float x_resolution, float y_resolution, float z_resolution)
{
  // use ROS VoxelGrid to resample
  pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
  sor.setDownsampleAllData (false);
  sor.setInputCloud(boost::make_shared<sensor_msgs::PointCloud2> (inputPC));
  sor.setLeafSize(x_resolution, y_resolution, z_resolution);
  sor.filter(outputPC);
}

//===============================================================================================
void resample(const sensor_msgs::PointCloud2 &inputPC, pcl::PointCloud<pcl::PointXYZ> &outputPC,  float x_resolution, float y_resolution, float z_resolution, uint32_t x_max, uint32_t y_max, uint32_t z_max, uint32_t estimated_number_of_output_points, uint8_t insertGridMidpoints_flag)
{
  // e.g. resolution is 0.1 -> scale is 10
  float x_scale = 1.0/x_resolution;
  float y_scale = 1.0/y_resolution;
  float z_scale = 1.0/z_resolution;
  
  // scale max values
  x_max = x_max*x_scale;
  y_max = y_max*y_scale;
  z_max = z_max*z_scale;
  
  //======================= create grid and init with 0 ============================
  uint32_t x_width = 2*x_max + 1;
  uint32_t y_width = 2*y_max + 1;
  uint32_t z_width = z_max + 1;
  uint32_t xy_size = x_width*y_width;
  uint32_t grid_size = x_width*y_width*z_width;
  vector<bool> grid(grid_size, 0);
  
  //================================ copy/filter point data ========================
  // loop over inputPC points
  //   - ignore them if they are NaN, otherwise: round the point cooordinates to the grid cells
  //   - if the cell is already set, ignore the point, otherwise: set cell and insert grid cell mid point to outputPC
  
  // prepare outputPC
  outputPC.points.clear();
  outputPC.points.reserve (estimated_number_of_output_points);
  
  // tmps for loops
  float x = 0, y = 0, z = 0; // scaled point coordinates
  uint32_t x_idx, y_idx, z_idx, grid_idx;  // indices in 3d grid (shifted x and y coordinate (z_idx=z)), grid_idx is linearized index 
  pcl::PointXYZ *pt;
  uint32_t input_h = inputPC.height;
  uint32_t input_w = inputPC.width;
  const uint8_t *msg_data; // pointer to inputPC data
  
  uint32_t n_skip = 0, n_no_skip = 0;
  
  int32_t minus_x_max = - (int32_t)(x_max);
  int32_t minus_y_max = - (int32_t)(y_max);
  
  // decide if cell midpoints or input points should be stored
  if(insertGridMidpoints_flag)
  {
    // loop 
    for (uint32_t row = 0; row < input_h; ++row)
    {
      msg_data = &inputPC.data[row * inputPC.row_step]; // pointer to begining of row in inputPC
      for (uint32_t col = 0; col < input_w; ++col)
      {              
        // somehow ugly: get point by pointer type cast
        pt =  (pcl::PointXYZ*) (msg_data);
        
        // get coordinates and test if NaN
        x = pt->x;
        if( !isnan(x))
        {
          // protect from erroneous measurements (x_max etc. are already scaled!)
          if(  x > x_max || x < minus_x_max || y > y_max || y < minus_y_max || z > z_max || z < 0 )
          {
            n_skip++;
            continue;
          }
          
          x = round(pt->x * x_scale);
          y = round(pt->y * y_scale);
          z = round(pt->z * z_scale);
          
          // get grid index (grid orgin is at point [x_idx=x_max, y_idx=y_max, z_idx=0])
          x_idx = -(uint32_t)x + x_max;
          y_idx = -(uint32_t)y + y_max;
          z_idx = (uint32_t)z;
          
          grid_idx = z_idx*xy_size + y_idx*x_width + x_idx;
          
          if( grid[grid_idx] == 0 )
          {
            grid[grid_idx] = 1;
            // insert midpoint of grid cell
            outputPC.push_back( pcl::PointXYZ(x/x_scale, y/y_scale, z/z_scale) );
          }
          
        }
        
        msg_data += inputPC.point_step;
        
      }
    }
  }
  else
  {
    // loop 
    for (uint32_t row = 0; row < input_h; ++row)
    {
      msg_data = &inputPC.data[row * inputPC.row_step]; // pointer to begining of row in inputPC
      for (uint32_t col = 0; col < input_w; ++col)
      { 
        
        // somehow ugly: get point by pointer type cast
        pt =  (pcl::PointXYZ*) (msg_data);
        
        // get coordinates and test if NaN        
        x = pt->x; 
        if( !isnan(x))
        {
          x = round(pt->x * x_scale);
          y = round(pt->y * y_scale);
          z = round(pt->z * z_scale);

          // protect from erroneous measurements (x_max etc. are already scaled!)
          if(  x > x_max || x < minus_x_max || y > y_max || y < minus_y_max || z > z_max || z < 0 )
          {
            n_skip++;
            continue;
          }
          
          n_no_skip++;
          
          // get grid index (grid orgin is at point [x_idx=x_max, y_idx=y_max, z_idx=0])
          x_idx = -(uint32_t)x + x_max;
          y_idx = -(uint32_t)y + y_max;
          z_idx = (uint32_t)z;
          
          grid_idx = z_idx*xy_size + y_idx*x_width + x_idx;
          
          if( grid[grid_idx] == 0 )
          {
            grid[grid_idx] = 1;
            // insert point
            outputPC.push_back( *pt );
          }
          
        }
        
        msg_data += inputPC.point_step;
      }
    }
  }
    
  cout<<"skipped rate: "<<(float)n_skip/(n_skip+n_no_skip)<<endl;
    
  // ToDo: update output header?
  outputPC.width  = outputPC.points.size ();
  outputPC.height = 1;
  outputPC.header = inputPC.header;
  
}
