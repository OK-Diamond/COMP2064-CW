//
// Created by jamestbest on 3/27/25.
//

/**
 * locmap is an abstraction over the map providing locations with names instead of pose data
 *
 * locmap has two modes
 *  - Record: this is for giving names to goal locations
 *  - Publish: this is for publishing the known locations and listening for goto requests
 */

#include "Main.h"

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

// ros params
string  LOCATIONS_TOPIC,
        GOAL_TOPIC,
        GOTO_TOPIC,
        CANCEL_TOPIC;

locmap::LocMapLocations locations;

/**
 * Simple function to initialise the ros node, must be called before any ros calls
 */
static void init_ros(int argc, char** argv) {
    ros::init(argc, argv, "talker");
}

/**
 * Ros contains the parameters from the param.yaml file
 *  these are shared across the different nodes to allow easy changes
 * @param node The initialised ros NodeHandle
 */
static void setup_params(const ros::NodeHandle& node) {
    node.getParam("locations_dir"  , location_data_dir);
    node.getParam("LOCATIONS_TOPIC", LOCATIONS_TOPIC);
    node.getParam("GOTO_TOPIC"     , GOTO_TOPIC);
    node.getParam("GOAL_TOPIC"     , GOAL_TOPIC);
    node.getParam("END_GOAL_TOPIC" , CANCEL_TOPIC);
}

/**
 * Entry point function
 *  Will setup the ros node with its pubs and subs
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv) {
    string res;

    if (argc > 1 && (strcmp(argv[1], "pub") == 0)) {
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

/**
 * Create the two subscribers needed namely
 *  the goal subscriber which is used in record mode
 *  the goto subscriber which is used in publisher mode (for goto requests)
 * @param node
 */
static void create_subscribers(ros::NodeHandle node) {
    cout << "Creating listeners" << endl;
    goal_subscriber= node.subscribe(GOAL_TOPIC, 10, goal_callback);

    goto_subscriber= node.subscribe(GOTO_TOPIC, 1, goto_callback);
}

/**
 * Create the three publishers needed namely
 *  the location publisher which is used to publish all locmap known locations
 *  the end goal publisher which is used in record mode to cancel the goal when a user marks a location with a name
 *  the goal publisher which is used to send the pose information for the goto request to move_base
 * @param node
 */
static void create_publishers(ros::NodeHandle node) {
    cout << "Creating publishers" << endl;
    location_publisher= node.advertise<locmap::LocMapLocations>(LOCATIONS_TOPIC, 1, true);
    end_goal_publisher= node.advertise<actionlib_msgs::GoalID>(CANCEL_TOPIC, 1, false);

    goal_publisher= node.advertise<geometry_msgs::PoseStamped>(GOAL_TOPIC, 1, false);
}

/**
 * Called when the goto subscriber receives a request to go to a location
 *  the goal location just contains a location name which is checked against known locations
 * @param goal_location
 */
static void goto_callback(locmap::LocMapGoto goal_location) {
    locmap::LocMapLocation location;

    if (!find_location(goal_location.location_name.c_str(), &location)) {
        ROS_WARN("Unable to find requested location `%s`", goal_location.location_name.c_str());
        return;
    }

    if (record_mode) {
        return;
    }

    goal_publisher.publish(location.pose);
}

/**
 * Called when the goal subscriber receives a new goal
 *  this is only used in record mode
 * @param goal
 */
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

/**
 * publish all known locations
 *  only called at the start or when a new location is added
 */
static void publish_locations() {
    location_publisher.publish(locations);
}

/**
 * Find from the known locations the location that matches the name
 * @param name name of location to find
 * @param loc_out pointer to the LocMapLocation struct that will be set if found
 * @return if found location
 */
static bool find_location(string name, locmap::LocMapLocation* loc_out) {
    for (auto& loc: locations.locations) {
        if (loc.name == name) {
            *loc_out= loc;
            return true;
        }
    }

    return false;
}

/**
 * Add a new location to the known locations list
 *  update the published locations
 * @param location
 */
static void add_location(locmap::LocMapLocation location) {
    locations.locations.push_back(location);

    publish_locations();
}

/**
 * Locations include a group tag
 * @param group name of group
 * @return All locations within the group
 */
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

/**
 * Open a file for writing binary data
 * @param filename
 * @param ofs output file stream
 * @return
 */
static bool open_serialization_file(string filename, ofstream& ofs) {
    ofs.open(filename, ios::binary);

    if (!ofs.is_open()) {
        ROS_ERROR("Failed to open serialization file with filename %s", filename.c_str());
        return false;
    }

    return true;
}

/**
 * Open a file for reading binary data
 * @param file_path
 * @param ifs input file stream
 * @param size_out size of the file
 * @return
 */
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

/**
 * Deserialize the binary file that contains a location
 * @param file_path
 * @param location reference to the output location struct
 * @return
 */
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

/**
 * Deserialize all location files within the location data directory
 * @return
 */
static bool deserialize_all_files() {
    namespace efs= std::experimental::filesystem;
    vector<locmap::LocMapLocation> loc_vec;

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

/**
 * Write a location to the file system
 * @param location
 * @return
 */
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

/**
 * Debugging; shows the current working directory
 */
static void print_cwd() {
    char cwd[PATH_MAX];
    if (getcwd(cwd, sizeof(cwd)) != nullptr) {
        std::cout << "Cwd: " << cwd << std::endl;
    } else {
        perror("getcwd() error");
    }
}
