// 移动零
// 给定一个数组 nums，编写一个函数将所有 0 移动到数组的末尾，同时保持非零元素的相对顺序。
// 请注意 ，必须在不复制数组的情况下原地对数组进行操作。

#include <string>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <algorithm>

using namespace std;


// class Solution {
// public:
//     void moveZeroes(vector<int>& nums) {
//         int j = 0;
//         for(int i=0; i<nums.size(); i++)
//         {
//             if(nums[i] !=0)
//             {
//                 nums[j] = nums[i];
//                 j++;
//             }
//         }

//         for(int i=j; i<nums.size(); i++)
//         {
//             nums[i] = 0;
//         }
//     }
// };

class Solution {
public:
    void moveZeroes(vector<int>& nums) {
        int j = 0;
        for(int i=0; i<nums.size(); i++)
        {
            int tmp = nums[i];
            nums[i] = 0;
            if(tmp !=0)
            {
                nums[j] =tmp;
                j++;
            }
        }
    }
};


int main()
{
	Solution *mySolution = new Solution;
	vector<int> nums = {0,1,0,3,12};
	mySolution->moveZeroes(nums);

    for(int i=0; i<nums.size(); i++)
    {
        cout << nums[i]  << ",";
    }
    cout << endl;

    return 1;
}