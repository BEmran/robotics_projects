#include <iostream>
#include <list>
#include <map>
#include <queue>
#include <set>
#include <stack>
#include <unordered_map>
#include <vector>

template <typename T>
void Print(std::string msg, T con) {
  std::cout << msg << ": ";
  auto it = con.begin();
  for (; it != con.end(); it++) {
    std::cout << *it << " ";
  }
  std::cout << std::endl;
}

void Vector() {
  std::vector<int> vec{1, 2, 3};
  Print<std::vector<int>>("initialize", vec);
  vec.push_back(10);
  Print<std::vector<int>>("push back 10", vec);
  vec.insert(vec.begin() + 3, {4, 5, 6});
  Print<std::vector<int>>("insert at idx 3", vec);
  vec.erase(vec.begin(), vec.begin() + 2);
  Print<std::vector<int>>("erase idx 0 and 1", vec);
  vec.pop_back();
  Print<std::vector<int>>("pop back", vec);
}

void List() {
  std::list<int> list = {1, 2, 3, 4, 5};
  Print<std::list<int>>("initialize", list);
  list.push_back(6);
  Print<std::list<int>>("push back 6", list);
  list.push_front(0);
  Print<std::list<int>>("push front 0", list);
  list.pop_back();
  Print<std::list<int>>("pop back", list);
  list.pop_front();
  Print<std::list<int>>("pop front", list);
  std::list<int>::iterator it = list.begin();
  it++;
  it++;
  list.insert(it, {8, 9, 10});
  Print<std::list<int>>("insert at idx 2", list);
  list.erase(++list.begin());
  Print<std::list<int>>("erase idx 1", list);
  list.sort();
  Print<std::list<int>>("sort", list);
  list.reverse();
  Print<std::list<int>>("reverse", list);
}
void Stack() {
  std::list<int> list{1, 2, 3, 4};
  std::stack<int, std::list<int>> stack{list};
  stack.pop();
  stack.push(5);
  stack.top();
}
void Queue() {
  std::list<int> list{1, 2, 3, 4};
  std::queue<int, std::list<int>> queue{list};
  queue.pop();
  queue.push(5);
  queue.front();
}
void Map() {
  std::map<std::string, int> map{{"one", 1}, {"two", 2}, {"three", 3}};
  auto ptr = map.find("one");
}

void Set() {
  std::set<int> set{1, 2, 3, 4};
  set.find(1);
  set.insert(5);
}
int main(int argc, char const* argv[]) {
  Vector();
  List();
  return EXIT_SUCCESS;
}
