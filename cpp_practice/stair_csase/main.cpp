#include <iostream>
#include <vector>

int NumWaysOptimal(const int n, std::vector<int> actions)
{
    std::vector<int> result(n, 0);
    for (size_t i = 1; i <= n; i++)
    {
        int tmp = 0;
        for (const auto& a : actions)
        {
            if (a == i)
                tmp++;
            else if (a < i)
                tmp += result[i - a - 1];
        }
        result[i - 1] = tmp;
    }
    return result[n - 1];
}

int NumWays(const int n, std::vector<int> actions)
{
    int tmp = 0;
    for (const auto& a : actions)
    {
        if (a == n)
            tmp++;
        else if (a < n)
            tmp += NumWays(n - a, actions);
    }
    return tmp;
}

std::ostream& operator<<(std::ostream& os, std::vector<int> vec)
{
    os << "[";
    for (const auto& e : vec)
        os << e << " ";
    os << "]";
    return os;
}

int main(int argc, char const* argv[])
{
    const int n = 1000;
    auto x = {2, 3};
    std::cout << "number of steps= " << n << " number of actions: " << x
              << " possible ways: " << NumWaysOptimal(n, x) << " "
              << NumWays(n, x) << std::endl;

    return EXIT_SUCCESS;
}
