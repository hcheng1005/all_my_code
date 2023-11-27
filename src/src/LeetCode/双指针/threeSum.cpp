// 接雨水

#include <string>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <algorithm>

using namespace std;

// 超时
class Solution
{
public:
    vector<vector<int>> threeSum(vector<int> &nums)
    {

        sort(nums.begin(), nums.end());

        vector<vector<int>> res;

        for (int i1 = 0; i1 < nums.size(); i1++)
        {
            for (int i2 = 0; i2 < i1; i2++)
            {
                for (int i3 = i1 + 1; i3 < nums.size(); i3++)
                {
                    if ((nums[i2] + nums[i1] + nums[i3]) == 0)
                    {
                        bool same_ = false;
                        for (auto &res1 : res)
                        {
                            if ((nums[i2] == res1[0]) && (nums[i1] == res1[1]) && (nums[i3] == res1[2]))
                            {
                                same_ = true;
                                break;
                            }
                        }

                        if (!same_)
                        {
                            res.push_back({nums[i2], nums[i1], nums[i3]});
                        }
                    }
                }
            }
        }

        return res;
    }
};

int main()
{
    Solution *mySolution = new Solution;
    vector<int> nums = {-1, 0, 1, 2, -1, -4};
    vector<vector<int>> result = mySolution->threeSum(nums);

    for (auto &res1 : result)
    {
        for (auto &res2 : res1)
        {
            cout << res2 << ", ";
        }
        cout << endl;
    }

    return 1;
}