class Dijkstra:

    def __init__(self,graph,start,goal):
        self.graph = graph
        self.start = start
        self.goal = goal
        
        self.open_list = {}
        self.close_list = {}

        self.open_list[start] = 0.0
        
        self.parent = {start: None}
        self.min_dis = 0.0

        self.findDijkstra = True

    def shortest_path(self):
        while True:
            if not self.open_list:
                print("搜索结束，未搜索到路径")
                self.findDijkstra = False
                return False

            distance, min_node = min(zip(self.open_list.values(), self.open_list.keys()), key = lambda x: x[0])
            self.open_list.pop(min_node)
            self.close_list[min_node] = distance

            if min_node == self.goal:
                print("找到路径,Djijkstrak路径搜索结束")
                self.min_dis = self.close_list[self.goal]
                best_way = [self.goal]
                father_node = self.parent[self.goal]
                while father_node != self.start:
                    best_way.append(father_node)
                    father_node = self.parent[father_node]
                best_way.append(self.start)
                best_way = best_way[::-1]
                print("最佳路径为{},最短距离为{}".format(best_way, self.min_dis))
                return True

            for node in self.graph[min_node]:
                if node not in self.close_list:
                    if node in self.open_list:
                        if self.close_list[min_node] + self.graph[min_node][node] < self.open_list[node]:
                            self.open_list[node] = self.close_list[min_node] + self.graph[min_node][node]
                            self.parent[node] = min_node
                    else:
                        self.open_list[node] = self.close_list[min_node] + self.graph[min_node][node]
                        self.parent[node] = min_node


if __name__ == '__main__':
    g = {'1': {'2': 2, '4': 1},
         '2': {'4': 3, '5': 11},
         '3': {'1': 4, '6': 5},
         '4': {'3': 2, '6': 8, '7': 4, '5': 2},
         '5': {'7': 6},
         '7': {'6': 1}
         }

    start = '1'
    goal = '6'
    dijk = Dijkstra(g, start, goal)
    dijk.shortest_path()

