#include <algorithm>
#include <vector>
#include <string>
#include <iostream>
#include <unordered_map>
#include <set>
#include <sstream>
#include <cmath>

using namespace std;

bool isHappy(int n) {
    if (n == 1 or n == 7) return true;

    string s = to_string(n);
    
    while (s.length() > 1)
    {   
        int new_num = 0;
        for (char c : s)
        {
            int num = int(c) - int('0');
            new_num += pow(num, 2);
        }

        if (new_num == 1 or new_num == 7) return true;               

        s = to_string(new_num);
    }


    return false;
}

int main(){
    int n = 1111111;

    cout << isHappy(n) << endl;
    
    return 0;
}