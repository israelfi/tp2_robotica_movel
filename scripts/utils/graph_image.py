import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


class GraphImage:
    """
    Converts a image to a conncetivity graph
    """
    plot = True

    def __init__(self, map_dir: str, cell_size: float = 0.25, map_dims: list = [20, 20]) -> None:
        # Dimensões do mapa informado em metros (X, Y)
        self.map_dims = np.array(map_dims)

        # Tamanho das células
        self.cell_size = cell_size

        # Invertendo os valores para visualização (Branco - 0, Preto - 1)
        img = 1 - mpimg.imread(map_dir)

        # Square maze has dimension 3, but circular maze has only 2
        try:
            img = img[:, :, 0]
        except IndexError:
            pass

        # Apenas para garantir que só teremos esses dois valores
        threshold = 0.5
        img[img > threshold] = 1
        img[img<= threshold] = 0

        # Escala Pixel/Metro
        sy, sx = img.shape[:2] / self.map_dims

        rows, cols = (self.map_dims / self.cell_size).astype(int)
        self.grid = np.zeros((rows, cols))

        # Preenchendo o Grid
        # Cada célula recebe o somatório dos valores dos Pixels
        for r in range(rows):
            for c in range(cols):
                
                xi = int(c*self.cell_size*sx)
                xf = int(xi + self.cell_size*sx)
                
                yi = int(r*self.cell_size*sy)
                yf = int(yi + self.cell_size*sy)
                            
                self.grid[r, c] = np.sum(img[yi:yf,xi:xf])
                
        # Binarizando as células como Ocupadas (1) ou Não-ocupadas (0)       
        self.grid[self.grid > threshold] = 1
        self.grid[self.grid<= threshold] = 0    

         # Binarizando as células como Ocupadas (1) ou Não-ocupadas (0)       
        self.grid[self.grid > threshold] = 1
        self.grid[self.grid<= threshold] = 0        
        if self.plot:
            fig = plt.figure(figsize=(8,8), dpi=100)
            ax = fig.add_subplot(111, aspect='equal')

            # Plotando Mapa e Células
            obj = ax.imshow(img, cmap='Greys', extent=(0, map_dims[1], 0, map_dims[0]), origin='upper')
            obj = ax.imshow(self.grid, cmap='Reds', extent=(0, map_dims[1], 0, map_dims[0]), alpha=.6)

            # Plotando as linhas do grid para facilitar a visualização
            ax.grid(which='major', axis='both', linestyle='-', color='r', linewidth=1)
            ax.set_xticks(np.arange(0, map_dims[1]+1, cell_size))
            ax.set_yticks(np.arange(0, map_dims[0]+1, cell_size)) 
            plt.show()      

        # Criando vértices em todas as células
        self.G = nx.grid_2d_graph(rows, cols) 

        # Removendo células que estão em células marcas com obstáculos
        for r in range(rows):
            for c in range(cols):
                if self.grid[r][c] == 1:  
                    self.G.remove_node((r,c))

    def shortest_path(self, start_node: list, end_node: list, plot: bool = False):
       
        # Uses Dijkstra's algorithm
        path = nx.shortest_path(self.G, source=start_node, target=end_node, method="dijkstra")

        if plot:
            fig = plt.figure(figsize=(8,8), dpi=100)
            ax = fig.add_subplot(111, aspect='equal')
            
            # Os vértices serão plotados no centro da célula  
            pos = {node:(node[1]*self.cell_size+self.cell_size/2, self.map_dims[0]-node[0]*self.cell_size-self.cell_size/2) for node in self.G.nodes()}
            nx.draw_networkx_nodes(self.G, pos, nodelist=path, node_size=100, node_color='b')
            # Mapa
            obj = ax.imshow(self.grid, cmap='Greys', extent=(0, self.map_dims[1], 0,self.map_dims[0]))
            plt.show()
        return path

    
    def plot_path():
        pass
