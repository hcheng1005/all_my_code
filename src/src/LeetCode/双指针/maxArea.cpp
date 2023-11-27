// 盛最多水的容器
// 给定一个长度为 n 的整数数组 height 。有 n 条垂线，第 i 条线的两个端点是 (i, 0) 和 (i, height[i]) 。
// 找出其中的两条线，使得它们与 x 轴共同构成的容器可以容纳最多的水。
// 返回容器可以储存的最大水量。

#include <string>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <algorithm>

using namespace std;

// 暴力循环：提示运行超时
// class Solution
// {
// public:
//     int maxArea(vector<int> &height)
//     {
//         int tmp_s = 0, max_s = 0;
//         for (int i = 0; i < height.size()-1; i++)
//         {
//             for (int j = (i+1); j < height.size(); j++)
//             {
//                 tmp_s = (j-i) * (min(height[i], height[j]));
//                 max_s = max(tmp_s, max_s);
//             }
//         }

//         return max_s;
//     }
// };

// class Solution
// {
// public:
//     int maxArea(vector<int> &height)
//     {
//         int tmp_s = 0, max_s = 0;
//         int i = 0, j = height.size() - 1;
//         do
//         {
//             tmp_s = (j - i) * (min(height[i], height[j]));
//             max_s = max(tmp_s, max_s);

//             if (height[i] < height[j])
//             {
//                 i++;
//             }
//             else
//             {
//                 j--;
//             }

//         } while (i != j);


//         return max_s;
//     }
// };


class Solution {

public:

    int maxArea(vector<int>& height) {

        int i = 0, j = height.size() - 1, res = 0;

        while(i < j) {

            res = height[i] < height[j] ? 

                max(res, (j - i) * height[i++]): 

                max(res, (j - i) * height[j--]); 
        }

        return res;
    }
};


int main()
{
    Solution *mySolution = new Solution;
    vector<int> height = {1, 8, 6, 2, 5, 4, 8, 3, 7};
    int result = mySolution->maxArea(height);

    cout << result << endl;

    return 1;
}