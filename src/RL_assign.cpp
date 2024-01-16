#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include "../include/managers/RL_assign.hpp"
using namespace std;

RL_assign::RL_assign(vector<string>& inst_arr)
{
	arr = inst_arr;
}

vector<string> RL_assign::RH_arr() 
{
	vector<string> s_based_pos = { "CC_L", "HH", "HT", "MT", "FT", "R", "CC_R" };

    for (size_t i = 0; i < arr.size(); ++i) {
        std::vector<std::string> spl;
        // Split the string by '+'
        size_t pos = arr[i].find('+');
        if (pos != std::string::npos) {
            spl.push_back(arr[i].substr(0, pos));
            spl.push_back(arr[i].substr(pos + 1));
        }
        else {
            spl.push_back(arr[i]);
            spl.push_back("");
        }

        // size_t first_index = 0;

        if (arr[i] == "") {
            R_hnd.push_back("");
            L_hnd.push_back("");
        }

        if (arr[i] != "" && arr[i].find('+') != std::string::npos) {
            if (arr[i].find('S') != std::string::npos) {
                L_hnd.push_back("S");
                R_hnd.push_back(arr[i].substr(0, arr[i].find('S')).erase(arr[i].find('S'), 1).erase(arr[i].find('+'), 1));
            }
            else {
                int pos1 = std::distance(s_based_pos.begin(), std::find(s_based_pos.begin(), s_based_pos.end(), spl[0]));
                int pos2 = std::distance(s_based_pos.begin(), std::find(s_based_pos.begin(), s_based_pos.end(), spl[1]));
                if (pos1 < pos2) {
                    R_hnd.push_back(spl[1]);
                    L_hnd.push_back(spl[0]);
                }
                else {
                    R_hnd.push_back(spl[0]);
                    L_hnd.push_back(spl[1]);
                }
            }
            // first_index = i;
        }
        else if (arr[i] != "") {
            R_hnd.push_back(arr[i]);
            L_hnd.push_back("");
            // first_index = i;
        }
    }

    return R_hnd;
}


vector<string> RL_assign::LH_arr()
{    
    return L_hnd;
}