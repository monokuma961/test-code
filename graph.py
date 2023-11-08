# graph

class graph:
    def __init__(self):
        self.nodes={}
        self.edge={}
        self.neighbors={}
    
    def add_node(self, node, **attr):        
        if isinstance(node, (int,float,str)) is False:
            raise ValueError("node needs to be int, float, str")
        else:
            if node not in self.nodes.keys():
                self.nodes[node]={}
            self.nodes[node].update(attr)
    
    def add_edge(self, u, v, **attr):        
        if isinstance(u, (int,float,str)) is False or isinstance(v, (int,float,str)) is False:
            raise ValueError("u,v need to be int,float,str")
        else:
            #add nodes
            if u not in self.nodes.keys():
                if u is None:
                    raise ValueError("u cannot be None")
                self.nodes[u]={}
            if v not in self.nodes.keys():
                if v is None:
                    raise ValueError("v cannot be None")
                self.nodes[v]={}
            #add the edge
            self.edge[u]={}
            self.edge[u][v]={}
            self.edge[u][v].update(attr)
            self.edge[v]={}
            self.edge[v][u]={}
            self.edge[v][u].update(attr)
            #update neighbors
            if u not in self.neighbors.keys():
                self.neighbors[u]=[]
                self.neighbors[u].append(v)
            else:
                self.neighbors[u].append(v)
            if v not in self.neighbors.keys():
                self.neighbors[v]=[]
                self.neighbors[v].append(u)
            else:
                self.neighbors[v].append(u)
            
    
    def edges(self):
        
        return 0
            
            
                
if __name__ == '__main__':    
    graph1=graph()
    graph1.add_node(1)
    graph1.nodes[1]['snydrome']=1
    print(graph1.nodes)