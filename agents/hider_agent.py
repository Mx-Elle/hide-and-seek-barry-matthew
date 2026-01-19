# Barry
from agent_base import Agent
from world_state import WorldState
from Mesh.nav_mesh import NavMesh
import shapely
import pygame
import random
import math
from Mesh.nav_mesh import NavMeshCell
import sys
from queue import PriorityQueue as pq
from queue import Queue

class DumbHider(Agent):

    def __init__(self, world_map: NavMesh, max_speed: float):
        Agent.__init__(self, world_map, max_speed)
        self.name = "Dumb hider"
        self.target = None
        self.map = world_map
        self.max_speed = max_speed
        self._state: WorldState | None = None
        self.cnt = 0
        # self._lock = Lock()
        self._next_move: tuple[float, float] | None = None
        self.opq = []
        self.prevposition = None
        self.add_cost={}
        self.initial=False

    def astar(
        self, _, start_cell: NavMeshCell, end_cell: NavMeshCell
    ,state: WorldState) -> list[NavMeshCell] | None:

        # naming and initializing
        # priority queue on cost
        frontier = pq()

        came = {}
        came[start_cell] = None

        cost = {}
        cost[start_cell] = 0
        frontier.put(
            (0, random.randint(0, sys.maxsize), start_cell)
        )  # using random integers to avoid crashing
        # put score, tiebreaker, curr node

        while not frontier.empty():
            _, _, curr = frontier.get()
            # print(curr)
            # evaluate if reached end at the start of every while loop
            if curr == end_cell:
                # reset
                route = []
                temp = end_cell

                # find routes and append to route[]
                while temp != start_cell:
                    route.append(temp)
                    temp = came[temp]

                route.append(start_cell)
                return route[::-1]

            for nbs in curr.neighbors:
                if not self.map.has_line_of_sight(
                    curr.polygon.centroid, nbs.polygon.centroid
                ):
                    continue
                curr_cost = cost[curr] + curr.distance(nbs)
                #+ math.dist((self._state.seeker_position.x,self._state.seeker_position.y), (curr.polygon.centroid.x,curr.polygon.centroid.y))
                #-math.dist((curr.polygon.centroid.x,curr.polygon.centroid.y),(state.seeker_position.x,state.seeker_position.y))
                if curr in self.add_cost:
                    curr_cost+=self.add_cost[curr]
                if nbs not in cost or curr_cost <= cost[nbs]:  # cost best?
                    cost[nbs] = curr_cost
                    # total cost this node, random int tiebreaker, node stored
                    frontier.put(
                        (
                            curr_cost + nbs.distance(end_cell),
                            random.randint(0, sys.maxsize),
                            nbs,
                        )
                    )
                    came[nbs] = curr

        return None  # fail to find route
    def seeker_can_see(self, state: WorldState):
        cell_queue=[]
        answer_queue=[]
        visited={}
        cell_queue.append(self.map.find_cell(state.seeker_position))
        answer_queue.append(self.map.find_cell(state.seeker_position))
        while len(cell_queue)>0:
            tmp=cell_queue[0]
            visited[tmp]=True
            cell_queue.pop(0)
            for itm in tmp.neighbors:
                if itm in visited and visited[itm]==True: continue
                if self.map.has_line_of_sight(itm.polygon.centroid, state.seeker_position):
                    cell_queue.append(itm)
                    answer_queue.append(itm)
        self.add_cost={}
        for itm in answer_queue:
            self.add_cost[itm]=100000
            
    def far_cell(self, state: WorldState):
        cell_queue=[]
        visited={}
        max_cell=self.map.find_cell(state.seeker_position)
        cell_queue.append(self.map.find_cell(state.seeker_position))
        while len(cell_queue)>0:

            tmp=cell_queue[0]
            visited[tmp]=True
            cell_queue.pop(0)
            dist=math.dist((state.seeker_position.x,state.seeker_position.y),(tmp.polygon.centroid.x,tmp.polygon.centroid.y))
            dist_max=math.dist((state.seeker_position.x,state.seeker_position.y),(max_cell.polygon.centroid.x,max_cell.polygon.centroid.y))
            if dist>dist_max: max_cell=tmp
            for itm in tmp.neighbors:
                if itm in visited and visited[itm]==True: continue
                cell_queue.append(itm)              
        return max_cell    
    def act(self, state: WorldState) -> tuple[float, float] | None:
        #print(math.dist((state.hider_position.x,state.hider_position.y),(state.seeker_position.x,state.seeker_position.y)))
        #print(self.add_cost)
        #print(f"{self.target=}")
        dx=state.hider_position.x-state.seeker_position.x
        dy=state.hider_position.y-state.seeker_position.y
        if self.add_cost=={}:
            self.opq=[]
            self.target=self.far_cell(state).polygon.centroid
            print(self.target)
            self.seeker_can_see(state)
        else:
            tmp_add=[]
            tmp_del=[]
            for itm in self.add_cost:
                for nb in itm.neighbors:
                    if self.map.has_line_of_sight(nb.polygon.centroid, state.seeker_position) and nb not in self.add_cost and nb not in tmp_add:
                        tmp_add.append(nb)
                if not self.map.has_line_of_sight(itm.polygon.centroid, state.seeker_position):
                    if itm not in tmp_del: tmp_del.append(itm)
            for itm in tmp_add: self.add_cost[itm]=100000
            for itm in tmp_del: del self.add_cost[itm]
        self.cnt += 1
        #print(self.opq)
        if self.target is None or state.hider_position == self.target or self.target in self.add_cost:
            dx, dy = (
                0,0
            )
            self.target=self.map.random_position()
            while self.target==None or self.target in self.add_cost or math.dist((dx,dy),(0,0))<=500: 
                self.target = self.map.random_position()
                target=self.target
                dx, dy = (
                target.x - state.hider_position.x,
                target.y - state.hider_position.y,
            )
        # if self.prevposition == state.hider_position:
        #     self.target = self.map.random_position()
        self.prevposition = state.hider_position
        my_cell = self.map.find_cell(state.hider_position)
        target = self.target
        target_cell = self.map.find_cell(target)
        if state.hider_position == target or not self.map.in_bounds(target):
            self.target = None
            return (0,0)
        # print(2)
        if my_cell == target_cell or self.map.has_line_of_sight(
            state.hider_position, target
        ):
            # print("hider", state.hider_position)
            # print("easy")
            # print("target", target)
            # print("easy")
            self.target = None
            dx, dy = (
                target.x - state.hider_position.x,
                target.y - state.hider_position.y,
            )
            distance = math.dist((dx, dy), (0, 0))
            speed = min(distance, self.max_speed*0.9999999999)
            dx, dy = speed * dx / distance, speed * dy / distance
            #print((dx, dy))
            return (dx, dy)
        #print(3)
        if self.opq == [] and my_cell != target_cell:
            # print('yayyy')
            tmp = self.astar(self, my_cell, target_cell,self._state)
            if tmp == None:
                return (0, 0)
            self.opq = tmp
        cnt = 0
        # print('test')

        while len(self.opq) > cnt:
            # print('matthew')
            if self.map.has_line_of_sight(
                state.hider_position, self.opq[cnt].polygon.centroid
            ):
                cnt += 1
            else:
                break
        cnt -= 1
        # print(4)
        # print(cnt)
        if state.hider_position == target or not self.map.in_bounds(target):
            self.opq = []
            ##print("yeah")
            return
        # print(5)
        if cnt > 0:
            for i in range(0, cnt):
                # print('pop')
                self.opq.pop(0)
        
        if not self.map.has_line_of_sight(
            state.hider_position, self.opq[0].polygon.centroid
        ):
            self.opq.pop(0)
            return
        dx=state.hider_position.x-state.seeker_position.x
        dy=state.hider_position.y-state.seeker_position.y
        dist_tmp=math.dist((0,0),(dx,dy))
        if self.opq[0] in self.add_cost and dist_tmp>40:
            self.opq=[]
            #print('matthew is the greatest########orzzzzzzzzzzzzz')
            return
        if (math.dist((0,0),(self.opq[0].polygon.centroid.x,self.opq[0].polygon.centroid.y))<=dist_tmp) and dist_tmp>40:
            self.opq=[]
            #print('matthew is the greatest########orzzzzzzzzzzzzz')
            return
        if math.dist((0,0),(dx,dy))>=500: return(0,0)
        if self.opq:
            dx, dy = (
                self.opq[0].polygon.centroid.x - state.hider_position.x,
                self.opq[0].polygon.centroid.y - state.hider_position.y,
            )
        else:
            dx, dy = (
                target.x - state.hider_position.x,
                target.y - state.hider_position.y,
            )
        distance = math.dist((dx, dy), (0, 0))
        speed = min(distance, self.max_speed*0.9999999999)
        # print(4)
        if distance > 0:
            dx, dy = speed * dx / distance, speed * dy / distance
        else:
            dx, dy = 0, 0
            if self.opq:
                self.opq.pop(0)
        # print(f"{dx=}, {dy=}")
        return (dx, dy)

    @property
    def is_seeker(self) -> bool:
        return False