// 接雨水

#include <string>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <algorithm>

using namespace std;

// 提示超时
// class Solution
// {
// public:
//     int trap(vector<int> &height)
//     {
//         int res = 0;

//         for (int i = 1; i < height.size() - 1; i++)
//         {
//             int max_left = 0, max_right = 0;
//             for (int i1 = 0; i1 < i; i1++)
//             {
//                 max_left = max(max_left, height[i1]);
//             }

//             for (int i2 = i+1; i2 < height.size(); i2++)
//             {
//                 max_right = max(max_right, height[i2]);
//             }

//             // cout << i << ", " << max_left << ", " << max_right << ", " << height[i] << endl;

//             if ((max_left > height[i]) && (max_right > height[i]))
//             {
//                 res += (min(max_left, max_right) - height[i]);

//                 // cout << res << endl;
//             }
//         }

//         return res;
//     }
// };

// // 时间复杂度 O(n)
// // 空间复杂度 O(n)
// class Solution
// {
// public:
//     int trap(vector<int> &height)
//     {
//         int res = 0;
//         vector<int> max_left(height.size(), 0);
//         vector<int> max_right(height.size(), 0);

//         for (int i = 1; i < height.size() - 1; i++)
//         {
//             max_left[i] = max(max_left[i - 1], height[i - 1]);
//         }

//         for (int i = height.size() - 2; i >= 0; i--)
//         {
//             max_right[i] = max(max_right[i + 1], height[i + 1]);
//         }

//         for (int i = 1; i < height.size() - 1; i++)
//         {
//             if ((max_left[i] > height[i]) && (max_right[i] > height[i]))
//             {
//                 res += (min(max_left[i], max_right[i]) - height[i]);
//             }
//         }

//         return res;
//     }
// };

// 时间复杂度 O(n)
// 空间复杂度 O(1)
class Solution
{
public:
    int trap(vector<int> &height)
    {
        int res = 0;
        int max_left = 0, max_right = 0;
        int left_idx = 1, right_idx = height.size() - 2;

        for (int i = 1; i < height.size() - 1; i++)
        {
            if (height[left_idx - 1] < height[right_idx + 1]) // 左边短，继续更新左边
            {
                max_left = max(max_left, height[left_idx - 1]);
                int min_val = max_left;
                if (min_val > height[left_idx])
                {
                    res += (min_val - height[left_idx]);
                }
                left_idx++;
            }
            else
            {
                max_right = max(max_right, height[right_idx + 1]);
                int min_val = max_right;
                if (min_val > height[right_idx])
                {
                    res += (min_val - height[right_idx]);
                }
                right_idx--;
            }
        }

        return res;
    }
};

int main()
{
    Solution *mySolution = new Solution;
    vector<int> height = {0, 1, 0, 2, 1, 0, 1, 3, 2, 1, 2, 1};
    int result = mySolution->trap(height);

    cout << result << endl;

    return 1;
}