
#include <string>
#include <vector>
void TrimString(std::string& str);
int ParseLine(std::string& lines, std::vector<std::string>& splits);
void global_gps_dist_angle(double lati0, double logi0, double lati1, double logi1, float& dist, float& angle);
void global_dist_angle_to_gps(double lati0, double logi0, float dist, float angle, double& lati1, double& logi1);
int ParseTaskList(std::string items, std::vector<std::string>& splits);
int ParseFloatList(std::string items, std::vector<float>& vals);