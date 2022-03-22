#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <sys/types.h> 
#include <dirent.h>
using namespace std;
int readFileList(const std::string &folderPath,
                 std::vector<std::string> &vFileList)
{
    DIR *dp;
    struct dirent *dirp;
    if ((dp = opendir(folderPath.c_str())) == NULL)
    {
        return 0;
    }

    int num = 0;

    while ((dirp = readdir(dp)) != NULL)
    {
        std::string name = std::string(dirp->d_name);

        if (name != "." && name != "..")
        {
            vFileList.push_back(name);
            num++;
        }
    }
    closedir(dp);

    // std::sort(vFileList.begin(), vFileList.end());

    return num;
}

int main(){
    string dirName = "./images/"; 
    vector <string> nameList;
    readFileList(dirName, nameList);
    sort(nameList.begin(),nameList.end());
    for(vector <string>::iterator it = nameList.begin(); it != nameList.end(); it++){
        cout << *it << endl;
    }
    return 0;
}