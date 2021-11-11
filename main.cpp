#include <iostream>
#include <list>
#include <vector>

struct Node
{
    std::vector<Node*> childs;
    int index = -1;
    int cost = -1;
    Node(int idx) : index(idx) {}
};

std::vector<int> bfs(int n, int m, std::vector<std::vector<int>> edges, int s)
{
    std::vector<Node*> nodes;
    // initialize nodes llist with new node
    for (int i = 0; i < n; ++i)
        nodes.push_back(new Node(i));
    // add childs and parent info from each edge
    for (int i = 0; i < m; ++i)
    {
        int st = edges[i][0] - 1;
        int en = edges[i][1] - 1;
        nodes[st]->childs.push_back(nodes[en]);
    }
    // create a list of node to check
    std::list<Node*> list;
    std::vector<Node*> done_list;
    // add start point as first node to check
    list.push_back(nodes[s - 1]);
    nodes[s - 1]->cost = 0;
    while (!list.empty())
    {
        Node* tmp = list.front();
        list.pop_front(); // remove it from list
        for (int i = 0; i < tmp->childs.size(); ++i)
        {
            tmp->childs[i]->cost = tmp->cost + 6;
            list.push_back(tmp->childs[i]);
        }
        if (tmp != nodes[s - 1])
            done_list.push_back(tmp);
    }
    std::vector<int> result(n - 1, -1);
    for (int i = 0; i < done_list.size(); ++i)
    {
        result[done_list[i]->index] = done_list[i]->cost;
    }
    return result;
}

int main(int argc, char const* argv[])
{
    int n = 5;
    std::vector<std::vector<int>> edges = {{5, 3}, {1, 2}, {1, 3}, {3, 4}};
    int m = edges.size();
    int s = 3;
    auto result = bfs(n, m, edges, s);
    for (auto r : result)
        std::cout << r << " ";
    std::cout << std::endl;
    /* code */
    return 0;
}
