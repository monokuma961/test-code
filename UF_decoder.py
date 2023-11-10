# UF_decoder

import networkx as nx
import numpy as np

def spanning_tree(G):
    '''
    

    Parameters
    ----------
    G : a Graph object of networkx.classes.graph module
        G is a connected graph.

    Returns
    -------
    sub_G : TYPE
        DESCRIPTION.

    '''
    
    node_number=len(G.nodes())

    k=0
    for n in G.nodes():
        G.nodes[n]['visited']=False
        if G.nodes[n]['boundary']==True:
            k+=1
    if k > 0:
        node_number=node_number-k+1
    
    tree_nodes=[]#the node whose 'visited' is True
    tree_edges=[]#the edges of tree
    nodes=[]
    boundary=0#make sure the subgraph only has at most one boundary node
    for n in G.nodes():
        if G.nodes[n]['visited']==True:
            continue
        else:
            #s=n#the root
            if G.nodes[n]['boundary']==1:
                continue
            else:
                G.nodes[n]['visited']=True
                tree_nodes.append(n)
                nodes.append(n)
                break

    while True:
        for n in tree_nodes:
            neighbor=nx.neighbors(G, n)
            for node in neighbor:
                if G.nodes[node]['visited'] == False:
                    if boundary*G.nodes[node]['boundary']==1:
                        continue
                    else:
                        tree_edges.append((n,node))#these edges don't have extra information
                        tree_nodes.append(node)
                        G.nodes[node]['visited']=True
                        if boundary == 0 and G.nodes[node]['boundary'] == 1:
                            boundary+=1
                            continue
                        else:
                            continue
                else:
                    continue
        nodes=tree_nodes#iterative

        if len(tree_nodes) == node_number:
            break
        
    sub_G=nx.Graph()
    sub_G.add_edges_from(tree_edges)
    
    
    return sub_G
    
    

def erasure_decoder(G,E):#the G is the graph of the syndrome. the E is the erasure edge. the V is the marked vertex(not including boundaries)
    A=[]#the set of error edge
    
    connected_graph=G.subgraph(E)#use E to construct the connected graph
    
    tree=spanning_tree(connected_graph)#get spanning tree
    
    
    while True:
        nodes=list(tree.nodes())
        for node in nodes:
            if G.nodes[node]['boundary'] == 0:#make sure the vertex is not the boundary vertex
                if tree.degree(node) == 1:
                    
                    neighbor=nx.neighbors(tree,node)
                    for n in neighbor:
                        neighbor_node=n
                    tree.remove_edge(node,neighbor_node)#remove the edge e.
                    tree.remove_node(node)
                    if G.nodes[node]['syndrome'] == 1:
                        A.append((node,neighbor_node))#add edge e to error edge
                        G.edges[(node,neighbor_node)]['syndrome']=(G.edges[(node,neighbor_node)]['syndrome']+1)%2
                        G.nodes[neighbor_node]['syndrome']=(G.nodes[neighbor_node]['syndrome']+1)%2
                    else:
                        continue
            else:
                continue
        
        if tree.number_of_edges() == 0:
            break
    
    return A,G

class Union_Find:#UF_decoder:
    
    def __init__(self):
        '''
        

        Returns
        -------
        None.

        '''
        self.cluster={}#the cluster
        self.clusters_tree={}#to search the root of node
        self.Support={}#the state of the edge
        self.Boundary={}#the boundary node of the cluster
        self.root={}#store the properties of cluster in the root

    
    
    
    def Grow(self, graph, L):
        '''
        

        Parameters
        ----------
        G : TYPE
            DESCRIPTION.
        L : TYPE
            DESCRIPTION.
        Boundary : TYPE
            DESCRIPTION.
        Support : TYPE
            DESCRIPTION.

        Returns
        -------
        F : TYPE
            DESCRIPTION.
        Support : TYPE
            DESCRIPTION.

        '''
        
        
        F=[]
        while len(F) == 0:
            for root in L:
                self.boundary=self.Boundary[root]
                #new_boundary={}
                for node in self.boundary:
                    edges=list(graph.edges(node))
                    new_edges=[]
                    for edge in edges:
                        #print(1)
                        node_1=min(edge)
                        node_2=max(edge)
                        if self.Support[(node_1,node_2)] != 1:
                            self.Support[(node_1,node_2)]+=0.5
                            new_edges.append((node_1,node_2))
                        else:
                            continue
                    for edge in new_edges:
                        if self.Support[edge] == 1:
                            F.append(edge)
                        else:
                            continue                    
        
        return F
    
    def Find(self,u):
        if u in self.clusters_tree.keys():
            root=self.clusters_tree[u]
            return root
        else:
            return None
        
    def Union(self,root_node_u,root_node_v):
        size_u=self.root[root_node_u][0]
        size_v=self.root[root_node_v][0]
        if size_v <= size_u:
            #update cluster
            for node in self.cluster[root_node_v]:
                self.clusters_tree[node]=root_node_u
                self.cluster[root_node_u].append(node)
            del self.cluster[root_node_v]
            #update root
            self.root[root_node_u][0]=size_u+size_v
            self.root[root_node_u][1]=((self.root[root_node_u][1]+self.root[root_node_v][1])%2)*self.root[root_node_u][2]*self.root[root_node_v][2]
            del self.root[root_node_v]
            #update Boundary
            for node in self.Boundary[root_node_v]:
                self.Boundary[root_node_u].append(node)
            del self.Boundary[root_node_v]
            
        else:
            for node in self.cluster[root_node_u]:
                self.clusters_tree[node]=root_node_v
                self.cluster[root_node_v].append(node)
            del self.cluster[root_node_u]
            self.root[root_node_v][0]=size_u+size_v
            self.root[root_node_v][1]=((self.root[root_node_u][1]+self.root[root_node_v][1])%2)*self.root[root_node_u][2]*self.root[root_node_v][2]
            del self.root[root_node_u]
            #update Boundary
            for node in self.Boundary[root_node_u]:
                self.Boundary[root_node_v].append(node)
            del self.Boundary[root_node_u]                
        
    
    def UF(self, graph):
        '''
        The UF is the main function of the algorithm. 

        Parameters
        ----------
        graph : graph is a Graph object of networkx.classes.graph module
            The snydrome graph.

        Returns
        -------
        None.

        '''
        L=[]
        
        for node in graph.nodes():
            if graph.nodes[node]['syndrome'] == 1: #and G.nodes[node]['boundary'] != 1:
                self.cluster[node]=[node]
                self.clusters_tree[node]=node
                self.root[node]=[1,1,1]
                self.Boundary[node]=[node]
                L.append(node)
        
        for edge in graph.edges():
            self.Support[edge]=0
        
        while len(L) > 0:
            
            #growing    
            F=Union_Find.Grow(self, graph, L)
            
            
            #merging
            for edge in F:
                #e=list(edge)
                u=edge[0]
                v=edge[1]
                root_node_u=Union_Find.Find(self,u)
                root_node_v=Union_Find.Find(self,v)
                #preprocessing
                if root_node_u is None:
                    self.clusters_tree[u]=root_node_v
                    self.cluster[root_node_v].append(u)
                    self.Boundary[root_node_v].append(u)
                    if graph.nodes[u]['boundary'] == 1:
                        self.root[root_node_v][2]=0
                elif root_node_v is None:
                    self.clusters_tree[v]=root_node_u
                    self.cluster[root_node_u].append(v)
                    self.Boundary[root_node_u].append(v)
                    if graph.nodes[v]['boundary'] == 1:
                        self.root[root_node_u][2]=0
                elif root_node_u == root_node_v:
                    pass
                
                else:
                    Union_Find.Union(self,root_node_u,root_node_v)
             
            #update L
            new_L=[]
            for j in range(len(L)):
                root_node=L.pop()
                new_root=Union_Find.Find(self,root_node)
                if new_root in L:
                    pass
                else:
                    new_L.append(new_root)
            L=new_L
            del new_L
            
            #remove the nodes that aren't the boundary nodes anymore
            for root_node in L:
                t=0
                for node in self.Boundary[root_node]:
                    edges=list(graph.edges(node))
                    for edge in edges:
                        node_1=min(edge)
                        node_2=max(edge)
                        if self.Support[(node_1,node_2)] != 1:
                            t+=1
                        else:
                            continue
                    if t == 0:
                        self.Boundary[root_node].remove(node)
            
            
            #updata the L accodrding to the parity and boundary
            new_L=[]
            for i in range(len(L)):
                root_node=Union_Find.Find(self,L[i])
                if root_node is not None:
                    if self.root[root_node][1]*self.root[root_node][2] == 1:
                        new_L.append(root_node)
                    else:
                        pass
                else:
                    pass
            L=new_L
            del new_L
        
        return graph
    
    def decoder(self,graph):
        syndrome_graph=Union_Find.UF(self, graph)

        error_edge=[]
        for node in self.cluster.keys():
            if len(self.cluster[node]) > 0:
                
                A,syndrome_graph=erasure_decoder(syndrome_graph,self.cluster[node])

                error_edge+=A
            else:
                continue
        
        return error_edge,syndrome_graph

if __name__ == '__main__':
    def syndrome_graph(d):
        number_vertices=d*(d+1)#including the virtual vertices
        
        syndrome_graph=nx.Graph()
        for i in range(number_vertices*d):
            syndrome_graph.add_node(i)
            if i%(d+1) == 0 or i%(d+1) == d:
                syndrome_graph.nodes[i]['boundary']=1
            else:
                syndrome_graph.nodes[i]['boundary']=0


        for i in range(d):
            for j in range(number_vertices):
                row=j//(d+1)+1
                column=j%(d+1)
                if row < d:
                    if column != d and column != 0:
                        syndrome_graph.add_edge(i*number_vertices+j,i*number_vertices+j+1)
                        syndrome_graph.add_edge(i*number_vertices+j,i*number_vertices+j+d+1)
                    elif column == 0:
                        syndrome_graph.add_edge(i*number_vertices+j,i*number_vertices+j+1)
                    else:
                        continue
                elif row == d:
                    if column != d:
                        syndrome_graph.add_edge(i*number_vertices+j,i*number_vertices+j+1)

        for i in range(d-1):
            for j in range(number_vertices):
                column=j%(d+1)
                if 0 < column < d:
                    syndrome_graph.add_edge(i*number_vertices+j,(i+1)*number_vertices+j)

        
        for edge in syndrome_graph.edges():
            syndrome_graph.edges[edge]['syndrome']=0
            
        for node in syndrome_graph.nodes():
            syndrome_graph.nodes[node]['syndrome']=0
            
        
        return syndrome_graph
    
    def circuit(G,d,p):
        number_dqubits=d*d+(d-1)**2
        number_mqubits=d**2-d
        
        data={}
        measure={}
        for i in range(number_dqubits):
            data[i]=0
        for i in range(number_mqubits):
            measure[i]=0
        
        for i in range(d):
            X_error=np.random.rand(number_dqubits)
            for j in range(number_dqubits):            
                if X_error[j] <= p:
                    data[j]=data[j]+1
                    data[j]=data[j]%2

                    m=j%(2*d-1)
                    n=j//(2*d-1)
                    if m < d :
                        G.nodes[i*d*(d+1)+n*(d+1)+m]['syndrome']+=1
                        G.nodes[i*d*(d+1)+n*(d+1)+m]['syndrome']=G.nodes[i*d*(d+1)+n*(d+1)+m]['syndrome']%2
                        G.nodes[i*d*(d+1)+n*(d+1)+m+1]['syndrome']+=1
                        G.nodes[i*d*(d+1)+n*(d+1)+m+1]['syndrome']=G.nodes[i*d*(d+1)+n*(d+1)+m+1]['syndrome']%2
                        G.edges[(i*d*(d+1)+n*(d+1)+m,i*d*(d+1)+n*(d+1)+m+1)]['syndrome']=(G.edges[(i*d*(d+1)+n*(d+1)+m,i*d*(d+1)+n*(d+1)+m+1)]['syndrome']+1)%2
                        #print(i*d*(d+1)+n*(d+1)+m)
                        #print(i*d*(d+1)+n*(d+1)+m+1)
                    else:
                        G.nodes[i*d*(d+1)+m-d+1+n*(d+1)]['syndrome']+=1
                        G.nodes[i*d*(d+1)+m-d+1+n*(d+1)]['syndrome']=G.nodes[i*d*(d+1)+m-d+1+n*(d+1)]['syndrome']%2
                        G.nodes[i*d*(d+1)+m+2+n*(d+1)]['syndrome']+=1
                        G.nodes[i*d*(d+1)+m+2+n*(d+1)]['syndrome']=G.nodes[i*d*(d+1)+m+2+n*(d+1)]['syndrome']%2
                        G.edges[(i*d*(d+1)+m-d+1+n*(d+1),i*d*(d+1)+m+2+n*(d+1))]['syndrome']=(G.edges[(i*d*(d+1)+m-d+1+n*(d+1),i*d*(d+1)+m+2+n*(d+1))]['syndrome']+1)%2
                        #print(i*d*(d+1)+m-d+1+n*(d+1))
                        #print(i*d*(d+1)+m+2+n*(d+1))
        
        for i in range(d-1):
            X_error=np.random.rand(number_mqubits)
            for j in range(number_mqubits):            
                if X_error[j] <= p:
                    measure[j]+=1
                    measure[j]=measure[j]%2
                    row=j//(d-1)
                    column=j%(d-1)
                    G.nodes[i*d*(d+1)+row*(d+1)+1+column]['syndrome']+=1
                    G.nodes[i*d*(d+1)+row*(d+1)+1+column]['syndrome']=G.nodes[i*d*(d+1)+row*(d+1)+1+column]['syndrome']%2
                    G.nodes[(i+1)*d*(d+1)+row*(d+1)+1+column]['syndrome']+=1
                    G.nodes[(i+1)*d*(d+1)+row*(d+1)+1+column]['syndrome']=G.nodes[(i+1)*d*(d+1)+row*(d+1)+1+column]['syndrome']%2
                else:
                    continue
        
        
        for i in range(d*d):
            G.nodes[i*(d+1)]['syndrome']=0
            G.nodes[i*(d+1)+d]['syndrome']=0
        
        return data,measure,G
    G=syndrome_graph(3)
    data,measure,G=circuit(G,3,0.01)

    c=Union_Find()
    error_edge,syndrome_graph=Union_Find.decoder(c, G)

        