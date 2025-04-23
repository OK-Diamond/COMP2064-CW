//
// Created by jamestbest on 3/27/25.
//

#include "Main.h"

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "actionlib_msgs/GoalID.h"

#include "locmap/LocMapLocations.h"
#include "locmap/LocMapLocation.h"
#include "locmap/LocMapGoto.h"

#include <fstream>
#include <string>

#include <experimental/filesystem>

static void add_location(locmap::LocMapLocation location);

using namespace std;
using namespace ros::serialization;

ros::Subscriber goal_subscriber;
ros::Publisher location_publisher;

ros::Publisher end_goal_publisher;

ros::Subscriber goto_subscriber;
ros::Publisher goal_publisher;

bool record_mode= false;

// this is just a default value for if not using the roslaunch, if that's the case then use rosrun in the same directory
string location_data_dir= "./location_data/";

locmap::LocMapLocations locations;

static bool open_serialization_file(string filename, ofstream& ofs) {
    ofs.open(filename, ios::binary);

    if (!ofs.is_open()) {
        ROS_ERROR("Failed to open serialization file with filename %s", filename.c_str());
        return false;
    }

    return true;
}

static bool open_deserialization_file(string file_path, ifstream& ifs, streamsize* size_out) {
    ifs.open(file_path, ios::binary | ios::ate);

    if (!ifs.is_open()) {
        ROS_ERROR("Failed to open deserialization file with file path %s", file_path.c_str());
        return false;
    }

    *size_out= ifs.tellg();
    ifs.seekg(0, ios::beg);

    return true;
}

static bool deserialize_file(string file_path, locmap::LocMapLocation& location) {
    ifstream ifs;

    streamsize f_size;
    if (!open_deserialization_file(file_path, ifs, &f_size)) {
        return false;
    }

    vector<uint8_t> buff(f_size);

    if (!ifs.read(reinterpret_cast<char*>(buff.data()), f_size)) {
        ROS_ERROR("Failed to read data from file");
        return false;
    }

    IStream stream(buff.data(), f_size);
    deserialize(stream, location);

    return true;
}

static bool deserialize_all_files() {
    namespace efs= std::experimental::filesystem;
    vector<locmap::LocMapLocation> loc_vec;

    // todo get the number of entries for the size of the vector

    for (const auto& file: efs::directory_iterator(location_data_dir)) {
        cout << "Deserializing " << file.path() << endl;

        locmap::LocMapLocation location;

        if (!deserialize_file(file.path(), location)) {
            ROS_ERROR("Unable to deserialize file %s", file.path().c_str());
            continue;
        }

        loc_vec.push_back(location);
    }

    locations.locations= loc_vec;
    locations.groups= vector<string>(0);

    return true;
}

static bool serialize_location(const locmap::LocMapLocation location) {
    ofstream ofs;
    const string file_path= location_data_dir + location.name;

    cout << "File path: " << file_path << endl;

    if (!open_serialization_file(file_path, ofs)) {
        return false;
    }

    uint32_t serial_size= serializationLength(location);
    vector<uint8_t> buff(serial_size);

    OStream stream(buff.data(), serial_size);
    serialize(stream, location);

    ofs.write(reinterpret_cast<char*>(buff.data()), serial_size);
    ofs.close();

    return true;
}

static void print_cwd() {
    char cwd[PATH_MAX];
    if (getcwd(cwd, sizeof(cwd)) != nullptr) {
        std::cout << "Cwd: " << cwd << std::endl;
    } else {
        perror("getcwd() error");
    }
}

static void goal_callback(geometry_msgs::PoseStamped goal) {
    if (!record_mode) {
        cout << "Ignored non-record mode goal" << endl;
        return;
    }

    cout << "Got new goal" << endl;

    actionlib_msgs::GoalID id= actionlib_msgs::GoalID();

    cout << "Publishing cancel goal" << endl;
    end_goal_publisher.publish(id);

    string loc_name;
    cout << "Enter location name: " << endl;
    getline(cin, loc_name);
    cout << "Location designated as: " << loc_name << endl;

    print_cwd();

    locmap::LocMapLocation location;
    location.pose= goal;
    location.name= loc_name;
    location.header= goal.header;
    location.group= "None";

    serialize_location(location);
    add_location(location);
}

static bool find_location(string name, locmap::LocMapLocation* loc_out) {
    cout << "Finding " << name << endl;
    for (auto& loc: locations.locations) {
        cout << "Checking against `" << loc.name << "`" << endl;
        if (loc.name == name) {
            *loc_out= loc;
            return true;
        }
    }

    return false;
}

static void goto_callback(locmap::LocMapGoto goal_location) {
    locmap::LocMapLocation location;

    if (!find_location(goal_location.location_name.c_str(), &location)) {
        ROS_WARN("Unable to find requested location `%s`", goal_location.location_name.c_str());
        return;
    }

    cout << "Found location: " << goal_location.location_name << endl;

    if (record_mode) {
        return;
    }

    goal_publisher.publish(location.pose);
}

static void create_subscribers(ros::NodeHandle node) {
    cout << "Creating listeners" << endl;
    goal_subscriber= node.subscribe("/move_base_simple/goal", 10, goal_callback);

    goto_subscriber= node.subscribe("/locmap/goto", 1, goto_callback);
}

static void create_publishers(ros::NodeHandle node) {
    cout << "Creating publishers" << endl;
    location_publisher= node.advertise<locmap::LocMapLocations>("/locmap/locations", 1, true);
    end_goal_publisher= node.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1, false);

    goal_publisher= node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, false);
}

static void publish_locations() {
    location_publisher.publish(locations);
}

static void add_location(locmap::LocMapLocation location) {
    locations.locations.push_back(location);

    publish_locations();
}

static locmap::LocMapLocations get_all_in_group(string group) {
    locmap::LocMapLocations location_ret;

    location_ret.groups= {group};

    location_ret.locations= vector<locmap::LocMapLocation>(locations.locations.size());

    for (const auto& loc: locations.locations) {
        if (loc.group == group)
            location_ret.locations.push_back(loc);
    }

    return location_ret;
}

static void setup_params(ros::NodeHandle node) {
    node.getParam("locations_dir", location_data_dir);
}

static void init_ros(int argc, char** argv) {
    ros::init(argc, argv, "talker");
}

int main(int argc, char** argv) {
    string res;

    if ((strcmp(argv[1], "pub") == 0)) {
        record_mode= false;
        goto main_init;
    }

    cout << "Record? (y/n): ";
    getline(cin, res);

    if (res == "y") {
        record_mode= true;
    }

main_init:
    init_ros(argc, argv);

    const ros::NodeHandle node;

    setup_params(node);

    cout << "DIR " << location_data_dir << endl;

    create_subscribers(node);
    create_publishers(node);

    if (deserialize_all_files()) {
        publish_locations();
    }

    cout << locations << endl;

    ros::spin();

    return EXIT_SUCCESS;
}