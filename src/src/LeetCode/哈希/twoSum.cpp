// 两数之和

#include <string>
#include <iostream>
#include <vector>
#include <unordered_map>

using namespace std;

// // way 1： for循环暴力求解
// class Solution {
// public:
//     vector<int> twoSum(vector<int>& nums, int target) {
//         for(int i1=0; i1<nums.size(); i1++)
//         {
//             for(int i2=(i1+1); i2<nums.size(); i2++)
//             {
//                 if((nums[i1] + nums[i2] ) == target)
//                 {
//                     return {i1, i2};
//                 }
//             }
//         }

//         return {};
//     }
// };

// // way 2： hash表
// class Solution {
// public:
//     vector<int> twoSum(vector<int>& nums, int target) {

//         unordered_map<int, int> hash_table;
//         for(int i1=0; i1<nums.size(); i1++)
//         {
//             auto it = hash_table.find(target-nums[i1]);

//             if(it == hash_table.end())
//             {
//                 hash_table[nums[i1]] = i1;
//             }
//             else
//             {
//                 return {i1, it->second};
//             }
//         }

//         return {};
//     }
// };

// // way 3： 
class Solution {
public:
    vector<int> twoSum(vector<int>& nums, int target) {
        unordered_map<int, int> map;
        for(int i = 0; i < nums.size(); ++i) {
            int n = target - nums[i];
            if(map.count(n)) {
                return {map[n], i};
            }
            map.emplace(nums[i], i);
        }
        return {};
    }
};

int main()
{
	Solution *mySolution = new Solution;
	vector<int> nums = {3,2,4};
    int target = 6;
	vector<int>  result = mySolution->twoSum(nums, target);

    cout << result[0] << ", " << result[1] << endl;

    return 1;
}