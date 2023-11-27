// 最长连续序列
// 给定一个未排序的整数数组 nums ，找出数字连续的最长序列（不要求序列元素在原数组中连续）的长度。
// 请你设计并实现时间复杂度为 O(n) 的算法解决此问题。

#include <string>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <algorithm>

using namespace std;

class Solution
{
public:
	int longestConsecutive(vector<int> &nums)
	{

		sort(nums.begin(), nums.end()); // 先排序

		if (nums.size() <= 1)
		{
			return nums.size();
		}
		else
		{
			int tmp = 0, max_len = 0;
			for (int i = 0; i < nums.size() - 1; i++)
			{
				if (abs(nums[i + 1] == (nums[i] + 1)))
				{
					tmp++;
				}
				else
				{
					if (nums[i + 1] != nums[i])
					{
						max_len = max(tmp, max_len);
						tmp = 0;
					}
				}
			}

			max_len = max(tmp, max_len);

			return (max_len + 1);
		}
	}
};

int main()
{
	Solution *mySolution = new Solution;
	vector<int> nums = {0, 3, 7, 2, 5, 8, 4, 6, 0, 1};
	int result = mySolution->longestConsecutive(nums);

	cout << result << endl;
	return 1;
}