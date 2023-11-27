// 字母异位词分组


#include <string>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <algorithm>

using namespace std;

class Solution
{
public:
	vector<vector<string>> groupAnagrams(vector<string> &strs)
	{
		unordered_map<string, vector<string>> mp;
		for (string &str : strs)
		{
			string key = str;
			sort(key.begin(), key.end());
			mp[key].emplace_back(str);
		}
		
		vector<vector<string>> ans;
		for (auto it = mp.begin(); it != mp.end(); ++it)
		{
			ans.emplace_back(it->second);
		}
		return ans;
	}
};


int main()
{
	Solution *mySolution = new Solution;
	vector<string> strs = {"eat", "tea", "tan", "ate", "nat", "bat"};
	vector<vector<string>>  result = mySolution->groupAnagrams(strs);

    for(const auto &str_vec:result)
    {
        for(const auto &str:str_vec)
        {
            cout << str;
        }
        cout << endl;
    }

    return 1;
}