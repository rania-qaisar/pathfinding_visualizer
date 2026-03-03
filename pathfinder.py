import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Button
import numpy as np
from collections import deque
import heapq, random, time
# ── Grid constants ────────────────────────────
ROWS, COLS = 10, 10
DIRS = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

def ok(r,c):     return 0<=r<ROWS and 0<=c<COLS
def free(g,r,c): return ok(r,c) and g[r][c]!=-1

# ── Grid generators ───────────────────────────
def random_grid(S,T, prob=0.25):
    g=np.zeros((ROWS,COLS),dtype=int)
    for r in range(ROWS):
        for c in range(COLS):
            if (r,c) not in(S,T) and random.random()<prob:
                g[r][c]=-1
    return g

def best_case_grid(S,T):
    """Very few walls — algorithm finds target quickly."""
    g=np.zeros((ROWS,COLS),dtype=int)
    # Only a couple of random walls far from S
    for r in range(3,ROWS):
        for c in range(3,COLS):
            if (r,c) not in(S,T) and random.random()<0.08:
                g[r][c]=-1
    return g

def worst_case_grid(S,T):
    """Dense walls + vertical barrier — algorithm struggles."""
    g=np.zeros((ROWS,COLS),dtype=int)
    # Dense random walls
    for r in range(ROWS):
        for c in range(COLS):
            if (r,c) not in(S,T) and random.random()<0.35:
                g[r][c]=-1
    # Vertical wall barrier in the middle with one gap
    mid=COLS//2
    gap=random.randint(1,ROWS-2)
    for r in range(ROWS):
        if r!=gap and (r,mid) not in(S,T):
            g[r][mid]=-1
    return g

print('Grid helpers ready!')

# ── All 6 Algorithms ──────────────────────────

def bfs(g,S,T):
    q=deque([[S]]); seen={S}
    while q:
        path=q.popleft(); node=path[-1]
        if node==T: yield seen,set(),path; return
        for dr,dc in DIRS:
            nb=(node[0]+dr,node[1]+dc)
            if free(g,*nb) and nb not in seen:
                seen.add(nb); q.append(path+[nb])
        yield seen.copy(),{p[-1] for p in q},None
    yield seen,set(),None

def dfs(g,S,T):
    stack=[[S]]; seen=set()
    while stack:
        path=stack.pop(); node=path[-1]
        if node in seen: continue
        seen.add(node)
        if node==T: yield seen,set(),path; return
        for dr,dc in DIRS:
            nb=(node[0]+dr,node[1]+dc)
            if free(g,*nb) and nb not in seen:
                stack.append(path+[nb])
        yield seen.copy(),set(),None
    yield seen,set(),None

def ucs(g,S,T):
    pq=[(0,0,[S])]; seen={}; ctr=1
    while pq:
        cost,_,path=heapq.heappop(pq); node=path[-1]
        if node in seen: continue
        seen[node]=cost
        if node==T: yield set(seen),set(),path; return
        for dr,dc in DIRS:
            nb=(node[0]+dr,node[1]+dc)
            step=1.414 if(dr and dc) else 1.0
            if free(g,*nb) and nb not in seen:
                heapq.heappush(pq,(cost+step,ctr,path+[nb])); ctr+=1
        yield set(seen),{x[2][-1] for x in pq},None
    yield set(seen),set(),None

def dls(g,S,T,lim=12):
    def go(path,d,vis):
        node=path[-1]
        if node==T: return path
        if d==0: return None
        vis.add(node)
        for dr,dc in DIRS:
            nb=(node[0]+dr,node[1]+dc)
            if free(g,*nb) and nb not in vis:
                r=go(path+[nb],d-1,vis)
                if r: return r
        vis.discard(node); return None
    vis=set(); res=go([S],lim,vis)
    yield vis,set(),res

def iddfs(g,S,T,max_d=30):
    all_seen=set()
    for depth in range(max_d+1):
        stack=[([S],0)]; vis=set()
        while stack:
            path,d=stack.pop(); node=path[-1]
            if node in vis: continue
            vis.add(node); all_seen.add(node)
            if node==T: yield all_seen,set(),path; return
            if d<depth:
                for dr,dc in DIRS:
                    nb=(node[0]+dr,node[1]+dc)
                    if free(g,*nb) and nb not in vis:
                        stack.append((path+[nb],d+1))
            yield all_seen.copy(),set(),None
    yield all_seen,set(),None

def bidir(g,S,T):
    if S==T: yield {S},set(),[S]; return
    fq=deque([[S]]); bq=deque([[T]])
    fv={S:[S]};      bv={T:[T]}
    while fq or bq:
        for q,vis,other in [(fq,fv,bv),(bq,bv,fv)]:
            if not q: continue
            path=q.popleft(); node=path[-1]
            for dr,dc in DIRS:
                nb=(node[0]+dr,node[1]+dc)
                if free(g,*nb) and nb not in vis:
                    vis[nb]=path+[nb]; q.append(path+[nb])
                    if nb in other:
                        f=fv[nb] if vis is fv else other[nb]
                        b=bv[nb] if vis is bv else other[nb]
                        yield set(fv)|set(bv),set(),f+list(reversed(b[1:]))
                        return
            yield set(fv)|set(bv),set(),None
    yield set(fv)|set(bv),set(),None

ALGOS={'BFS':bfs,'DFS':dfs,'UCS':ucs,'DLS':dls,'IDDFS':iddfs,'Bidirectional':bidir}
print('All 6 algorithms ready!')

# ── Main App ──────────────────────────────────
# ── GUI Application ──
class App:
    def __init__(self):
        self.S=(0,0); self.T=(ROWS-1,COLS-1)
        self.grid   = random_grid(self.S,self.T)
        self.seen   = set(); self.front=set()
        self.path   = []; self.steps=0
        self.t0     = 0;   self.running=False
        self.algo   = ''
        self._build()
        self._draw()
        plt.show()

    # ─────────────────────────────────────────
    def _build(self):
        self.fig=plt.figure(figsize=(16,9))
        self.fig.patch.set_facecolor('#1e1e2e')
        self.fig.canvas.manager.set_window_title('GOOD PERFORMANCE TIME APP')

        # Grid
        self.ax=self.fig.add_axes([0.02,0.05,0.55,0.92])
        self.ax.set_facecolor('#2a2a3e'); self.ax.axis('off')

        # Stats box
        ax_st=self.fig.add_axes([0.59,0.80,0.39,0.17])
        ax_st.set_facecolor('#2a2a3e'); ax_st.axis('off')
        ax_st.set_title('Stats', color='#fcc419', fontsize=10, fontweight='bold')
        self.st=ax_st.text(0.04,0.80,
            'Click an algorithm button to start',
            transform=ax_st.transAxes,
            color='white', fontsize=9, fontfamily='monospace', va='top')

        # ── 6 Algorithm blocks (each has: run | best | worst)
        algo_colors={
            'BFS':'#3498db','DFS':'#e74c3c','UCS':'#27ae60',
            'DLS':'#9b59b6','IDDFS':'#e67e22','Bidirectional':'#1abc9c'
        }
        self._btns=[]   # keep references!

        names=list(ALGOS.keys())
        for i,name in enumerate(names):
            col=algo_colors[name]
            y=0.66 - i*0.115

            # Main run button
            ax_r=self.fig.add_axes([0.59, y, 0.20, 0.09])
            b_r=Button(ax_r, f'● {name}', color=col, hovercolor='#ecf0f1')
            b_r.label.set_color('white')
            b_r.label.set_fontsize(9)
            b_r.label.set_fontweight('bold')
            b_r.on_clicked(lambda e,n=name: self._run(n, self.grid.copy()))
            self._btns.append(b_r)

            # Best case button
            ax_b=self.fig.add_axes([0.80, y+0.045, 0.09, 0.040])
            b_b=Button(ax_b,'BEST',color='#27ae60',hovercolor='#abebc6')
            b_b.label.set_color('white'); b_b.label.set_fontsize(7.5)
            b_b.label.set_fontweight('bold')
            b_b.on_clicked(lambda e,n=name: self._run_scenario(n,'best'))
            self._btns.append(b_b)

            # Worst case button
            ax_w=self.fig.add_axes([0.90, y+0.045, 0.09, 0.040])
            b_w=Button(ax_w,'WORST',color='#e74c3c',hovercolor='#f1948a')
            b_w.label.set_color('white'); b_w.label.set_fontsize(7.5)
            b_w.label.set_fontweight('bold')
            b_w.on_clicked(lambda e,n=name: self._run_scenario(n,'worst'))
            self._btns.append(b_w)

            # Small label
            ax_lbl=self.fig.add_axes([0.80, y, 0.19, 0.040])
            ax_lbl.set_facecolor('#1e1e2e'); ax_lbl.axis('off')
            ax_lbl.text(0.5,0.5,'▲ Best    ▲ Worst',
                        ha='center',va='center',
                        color='#aaaaaa',fontsize=7,
                        transform=ax_lbl.transAxes)

        # New Maze + Clear
        ax_nm=self.fig.add_axes([0.59,0.03,0.18,0.06])
        self.btn_nm=Button(ax_nm,'New Maze',color='#7f8c8d',hovercolor='#bdc3c7')
        self.btn_nm.label.set_color('white'); self.btn_nm.label.set_fontsize(8)
        self.btn_nm.label.set_fontweight('bold')
        self.btn_nm.on_clicked(self._new_maze)

        ax_cl=self.fig.add_axes([0.80,0.03,0.18,0.06])
        self.btn_cl=Button(ax_cl,'Clear',color='#c0392b',hovercolor='#e74c3c')
        self.btn_cl.label.set_color('white'); self.btn_cl.label.set_fontsize(8)
        self.btn_cl.label.set_fontweight('bold')
        self.btn_cl.on_clicked(self._clear)

    # ─────────────────────────────────────────
    def _draw(self, title=None):
        ax=self.ax; ax.clear()
        ax.set_facecolor('#2a2a3e')
        ax.set_xlim(-0.5,COLS-0.5)
        ax.set_ylim(-0.5,ROWS-0.5)
        ax.set_aspect('equal'); ax.invert_yaxis(); ax.axis('off')

        t=title if title else (self.algo if self.algo else 'Select an Algorithm')
        ax.set_title(t,fontsize=13,fontweight='bold',
                     color='#fcc419',pad=8)

        path_set=set(map(tuple,self.path))
        for r in range(ROWS):
            for c in range(COLS):
                cell=(r,c); v=self.grid[r][c]
                if   cell==self.S:          bg,lbl,tc='#27ae60','S','white'
                elif cell==self.T:          bg,lbl,tc='#2980b9','T','white'
                elif v==-1:                 bg,lbl,tc='#c0392b','-1','white'
                elif cell in path_set:      bg,lbl,tc='#f39c12','0','white'
                elif cell in self.front:    bg,lbl,tc='#f1c40f','0','#333'
                elif cell in self.seen:     bg,lbl,tc='#5dade2','0','white'
                else:                       bg,lbl,tc='#bdc3c7','0','#333'

                rect=patches.FancyBboxPatch(
                    (c-0.44,r-0.44),0.88,0.88,
                    boxstyle='round,pad=0.04',
                    linewidth=0.8,edgecolor='#888',facecolor=bg)
                ax.add_patch(rect)
                fs=12 if lbl in('S','T') else 9
                ax.text(c,r,lbl,ha='center',va='center',
                        fontsize=fs,fontweight='bold',color=tc)

        # stats
        plen=len(self.path) if self.path else '—'
        t_e=f'{time.time()-self.t0:.3f}s' if self.t0 else '—'
        self.st.set_text(
            f'Algorithm  : {self.algo}\n'
            f'Steps      : {self.steps}\n'
            f'Explored   : {len(self.seen)}\n'
            f'Path Nodes : {plen}\n'
            f'Time       : {t_e}')

        self.fig.canvas.draw_idle()

    # ─────────────────────────────────────────
    def _run(self, name, grid):
        """Run algorithm on given grid with live animation."""
        if self.running: return
        self.running=True; self.algo=name
        self.seen=set(); self.front=set()
        self.path=[]; self.steps=0; self.t0=time.time()
        self.grid=grid

        g=grid.copy()
        g[self.S[0]][self.S[1]]=0
        g[self.T[0]][self.T[1]]=0

        for seen,front,path in ALGOS[name](g,self.S,self.T):
            self.seen=seen; self.front=front; self.steps+=1
            if path:
                self.path=path; self.running=False
                elapsed=time.time()-self.t0
                self._draw(
                    f'{name}  |  Path={len(path)}  '
                    f'Explored={len(seen)}  '
                    f'Steps={self.steps}  '
                    f'Time={elapsed:.3f}s')
                return
            self._draw()
            plt.pause(0.07)

        self.running=False
        self._draw(f'{name}  |  NO PATH FOUND  Steps={self.steps}')

    def _run_scenario(self, name, scenario):
        """Load best or worst case grid then run algorithm."""
        if self.running: return
        if scenario=='best':
            g=best_case_grid(self.S,self.T)
        else:
            g=worst_case_grid(self.S,self.T)
        self._run(name, g)

    def _new_maze(self,e=None):
        if self.running: return
        self.grid=random_grid(self.S,self.T)
        self.seen=set(); self.front=set()
        self.path=[]; self.steps=0; self.t0=0; self.algo=''
        self.st.set_text('New maze ready! Click an algorithm.')
        self._draw()

    def _clear(self,e=None):
        if self.running: return
        self.grid=np.zeros((ROWS,COLS),dtype=int)
        self.seen=set(); self.front=set()
        self.path=[]; self.steps=0; self.t0=0; self.algo=''
        self.st.set_text('Grid cleared! Click an algorithm.')
        self._draw()



app = App()
