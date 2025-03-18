#include <string>
#include <fstream>
#include <vector>
#include <utility> // std::pair
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream
#include <boost/filesystem.hpp>
#include <iostream>
#include <chrono> 
#include <iomanip>
#include <dead_reckoning/BicycleModelSimple.hpp>

void read_csv(std::string filename, int run_no){
    
    std::ifstream myFile(filename);

    if(!myFile.is_open()) throw std::runtime_error("Could not open file");
    
    std::string line, colname;
    std::vector<std::string> colnames; 
    int val;

    if(myFile.good())
    {
        std::getline(myFile, line);
        std::stringstream ss(line);
        while(std::getline(ss, colname, ',')){
            colnames.push_back(colname);
        }
    }
    
    double dt = 0;
    double vel = 0; 
    double steer = 0; 
    std::chrono::system_clock::time_point previous_time_point;
    bool first_time_flag = true;
    int count = 0;

    
    
    std::string file_path = "/home/catkin_ws/src/data_processing/data/online_plotting/run_no" + std::to_string(run_no) + "/offline_encoder_odom.csv";
    std::ofstream output_file(file_path); // Open the file for writing
    output_file << "x,y\n"; 

    double vel_scale = 0.35;
    double steer_offset = 6.7;
    double l1 = 0.1;
    double l2 = 0.7;
    double steer_scale = 0.9;
    std::vector<double> params = {l1, l2, steer_scale};
    auto model = BicycleModelSimple(params);

    while(std::getline(myFile, line) && count < 10000000)
    {   
        count++;
        std::stringstream ss(line);
        std::string date;
        std::getline(ss, date, ',');
        ss >> vel;
        vel *= vel_scale;
        ss.ignore();  // Skip the comma
        ss >> steer;
        steer = (steer-steer_offset) * M_PI / 180;

        std::istringstream iss(date);

        // Parse the timestamp
        std::tm tm = {};
        iss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");

        if (iss.fail()) {
            std::cerr << "Failed to parse time" << std::endl;
            return;
        }

        std::time_t time = std::mktime(&tm);
        if (time == -1) {
            std::cerr << "Error converting time" << std::endl;
            return;
        }

        int milliseconds = 0;
        // Check for fractional part (milliseconds)
        if (iss.peek() == '.') {
            iss.ignore(); // Ignore the dot
            int fractional_part = 0;
            iss >> fractional_part;  // Read the fractional part as integer
            milliseconds = fractional_part;  // Assuming fractional part is in milliseconds
        }

        // Create a system clock time_point from the parsed time
        auto time_point = std::chrono::system_clock::from_time_t(time);
        auto timestamp_with_ms = time_point + std::chrono::milliseconds(milliseconds);

        static bool first_time_flag = true;
        static std::chrono::system_clock::time_point previous_time_point;
        double dt = 0;

        if (first_time_flag) {
            first_time_flag = false;
            dt = 0;  // No time difference for the first timestamp
            previous_time_point = timestamp_with_ms;
        } else {
            dt = std::chrono::duration_cast<std::chrono::milliseconds>(timestamp_with_ms - previous_time_point).count() / 1000.0;
            previous_time_point = timestamp_with_ms;
        }

        model.update(vel, steer, dt);
        auto pose = model.get_state();
        double x = pose[0]; 
        double y = pose[1];

        std::cout << "steer: " << steer << " theta: " << pose[2] << " x: " << x <<std::endl; 
        output_file << x << ',' << y << '\n';

    }

    output_file.close();
    // Close file
    myFile.close();

}

int main() {

    int run_no = 279;
    std::string file_path = "/home/catkin_ws/src/data_processing/data/online_plotting/run_no" + std::to_string(run_no) + "/bag2csv.csv";
    std::cout << file_path << std::endl;

    read_csv(file_path, run_no);
    return 0;
}
