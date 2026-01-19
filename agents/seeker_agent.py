#matthew
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
from collections import deque
from typing import Set, List


class DumbSeeker(Agent):
    """
    i tried to evaluate the round in casework:
    case 1 - not yet seen the hider and patrol to find hider
    case 2 - seen the hider and has not lost hider. then catch directly
    case 3 - seen the hider and lost hider. travel to the last seen point and bfs the possible places the hider can be in and search all
    i tried to make the agent not circle back to original point but it sometimes still do
    """
    def __init__(self, world_map: NavMesh, max_speed: float):
        Agent.__init__(self, world_map, max_speed)
        self.name = "Dumb seeker"
        self.target = None
        self.map = world_map
        self.max_speed = max_speed
        self._state: WorldState | None = None
        self.cnt = 0
        self._next_move: tuple[float, float] | None = None
        self.opq = []
        self.prevposition = None
        self.last_seen = deque(maxlen=10)
        self.searched_points: Set[tuple[float, float]] = set()
        self.last_pathps: Set[tuple[float, float]] = set()
        self.can_see = False
        self.lost = True
        self.fail = 0
        self.bfs_positions = deque(maxlen=20)
        self.curr_bfs_tar = []
        self.index = 0
        self.last_hider_check = None
        self.stuckcnt = 0
        self.last_positions = deque(maxlen=10)

    def astar(self, _, start_cell: NavMeshCell, end_cell: NavMeshCell) -> list[NavMeshCell] | None:
        frontier = pq()
        came = {}
        came[start_cell] = None
        cost = {}
        cost[start_cell] = 0
        frontier.put((0, random.randint(0, sys.maxsize), start_cell))
        
        while not frontier.empty():
            _, _, curr = frontier.get()
            if curr == end_cell:
                route = []
                temp = end_cell
                while temp != start_cell:
                    route.append(temp)
                    temp = came[temp]
                route.append(start_cell)
                return route[::-1]

            for nbs in curr.neighbors:
                if not self.map.has_line_of_sight(curr.polygon.centroid, nbs.polygon.centroid):
                    continue
                curr_cost = cost[curr] + curr.distance(nbs)

                if nbs not in cost or curr_cost <= cost[nbs]:
                    cost[nbs] = curr_cost
                    frontier.put((curr_cost + nbs.distance(end_cell), random.randint(0, sys.maxsize), nbs))
                    came[nbs] = curr

        return None
    
    
    
    def decide(self, state: WorldState):
        self.search_map(state.seeker_position)
        
        curr_pos = (round(state.seeker_position.x, 1), round(state.seeker_position.y, 1))
        self.last_positions.append(curr_pos)
        
        if self.is_stuck():
            self.stuckcnt += 1
            if self.stuckcnt > 20:
                self.force_reset()
                return self.map.random_position()
        else:
            self.stuckcnt = 0
        
        #case1
        if not self.can_see:
            if self.curr_bfs_tar and self.index < len(self.curr_bfs_tar):
                target = self.curr_bfs_tar[self.index]
                self.index += 1
                return target
            else:
                self.curr_bfs_tar = []
                self.index = 0
                return self.search_path(state)
        
        
        #case2
        if self.last_seen and not self.lost:
            return self.last_seen[-1]
        
        #case3
        if self.last_seen and self.lost:
            if not self.curr_bfs_tar:
                self.curr_bfs_tar = self.bfs(state)
                self.index = 0
            
            if self.curr_bfs_tar and self.index < len(self.curr_bfs_tar):
                target = self.curr_bfs_tar[self.index]
                self.index += 1
                return target
            else:
                self.curr_bfs_tar = []
                self.index = 0
                return self.search_path(state)
        
        
        return self.map.random_position()
    
    
    def move(self, state: WorldState, target):
        currc = self.map.find_cell(state.seeker_position)
        tarc = self.map.find_cell(target)
        
        if not currc or not tarc:
            return None
        
        if currc == tarc or self.map.has_line_of_sight(state.seeker_position, target):
            self.opq = []
            dx = target.x - state.seeker_position.x
            dy = target.y - state.seeker_position.y
            dis = math.sqrt(dx*dx + dy*dy)
            if dis > 0:
                speed = min(self.max_speed * 0.95, dis)
                return (speed * dx / dis, speed * dy / dis)
                #print('move1')
            return (0, 0)
        
    
        if not self.opq:
            path = self.astar(self, currc, tarc)
            if path:
                self.opq = path
                self.recp(path)
            else:
                return None
        
        if self.opq:
            while self.opq and self.map.has_line_of_sight(state.seeker_position, self.opq[0].polygon.centroid):
                next_dis = state.seeker_position.distance(self.opq[0].polygon.centroid)
                if next_dis < 5:
                    self.opq.pop(0)
                else:
                    break
            
            
            if self.opq:
                nxp = self.opq[0].polygon.centroid
                dx = nxp.x - state.seeker_position.x
                dy = nxp.y - state.seeker_position.y
                dis = math.sqrt(dx*dx + dy*dy)
                if dis > 0:
                    spd = min(self.max_speed * 0.95, dis)
                    return (speed * dx / dis, spd * dy / dis)
        
        return None
    
    
    def see_hider(self, state: WorldState):
        saw_hider = False
        hiderp = None
        
        if saw_hider:
            if self.map.has_line_of_sight(state.seeker_position, hiderp):
                self.last_seen.append(hiderp)
                self.can_see = True
                self.lost = False
                self.curr_bfs_tar = []
                self.index = 0
                
                return True
            else:
                self.can_see = False
                
                if self.last_seen:
                    self.lost = True
                return False
        else:
            self.can_see = False
            if self.last_seen:
                self.lost = True
            return False
    
    
    def search_map(self, position: shapely.Point):
        cell_key = (round(position.x, 1), round(position.y, 1))
        
        if cell_key not in self.searched_points:
            self.searched_points.add(cell_key)
            self.fail = 0
            
        return list(self.searched_points)
    
    def recp(self, path: List[NavMeshCell]):
        self.last_pathps.clear()
        pathps = []
        for cell in path:
            
            centroid = cell.polygon.centroid
            pointkey = (round(centroid.x, 1), round(centroid.y, 1))
            self.last_pathps.add(pointkey)
            pathps.append(pointkey)
        for pointkey in pathps:
            self.searched_points.add(pointkey)
    
    
    def is_stuck(self):
        if len(self.last_positions) < 5:
            #print('here')
            return False
        
        uni_pos = set(self.last_positions)
        #print('unique')
        return len(uni_pos) < 3
    
    def force_reset(self):
        self.searched_points.clear()
        self.last_pathps.clear()
        self.fail = 0
        self.stuckcnt = 0
        self.curr_bfs_tar = []
        self.index = 0
        self.last_positions.clear()
    
    def search_path(self, state: WorldState):
        curr_key = (round(state.seeker_position.x, 1), round(state.seeker_position.y, 1))
        
        bestp = None
        best_scr = -999999
        trys = 0
        
        while trys < 150:
            point = self.map.random_position()
            pointkey = (round(point.x, 1), round(point.y, 1))
            
            if pointkey == curr_key:
                trys += 1
                continue
            
            if pointkey in self.last_pathps:
                trys += 1
                continue
            
            scr = 0
            
            if pointkey in self.searched_points:
                scr -= 5000
            
            min_dis_to_searched = float('inf')
            for searched_key in self.searched_points:
                searched_point = shapely.Point(searched_key[0], searched_key[1])
                distance = point.distance(searched_point)
                if distance < min_dis_to_searched:
                    min_dis_to_searched = distance
            
            min_dis_to_last_path = float('inf')
            for last_path_key in self.last_pathps:
                last_path_point = shapely.Point(last_path_key[0], last_path_key[1])
                dis = point.distance(last_path_point)
                if dis < min_dis_to_last_path:
                    min_dis_to_last_path = dis
            
            if min_dis_to_last_path > 0:
                scr += min_dis_to_last_path * 500
            
            scr += min_dis_to_searched * 100
            
            dis_to_curr = state.seeker_position.distance(point)
            if 150 < dis_to_curr < 400:
                scr += 500
            elif dis_to_curr < 80:
                scr -= 1000
            elif dis_to_curr > 500:
                scr -= dis_to_curr
            
            if scr > best_scr:
                best_scr = scr
                bestp = point
            
            trys += 1
        
        if bestp:
            bestpkey = (round(bestp.x, 1), round(bestp.y, 1))
            if bestpkey in self.last_pathps:
                self.fail += 1
                if self.fail > 15:
                    self.force_reset()
                    return self.map.random_position()
                return self.search_path(state)
            
            self.fail = 0
            return bestp
        
        self.force_reset()
        return self.map.random_position()
    
    def bfs(self, state: WorldState):
        if not self.last_seen:
            return []
        
        all_pos = []
        last_pos = self.last_seen[-1]
        start_cell = self.map.find_cell(last_pos)
        
        if start_cell:
            visited = set()
            dq = deque([(start_cell, 0)])
            
            while dq and len(all_pos) < 30:
                curr_cell, depth = dq.popleft()
                if depth > 8:
                    continue
                
                cent = curr_cell.polygon.centroid
                pointkey = (round(cent.x, 1), round(cent.y, 1))
                
                if pointkey not in self.searched_points:
                    all_pos.append(cent)
                
                for nbr in curr_cell.neighbors:
                    if nbr not in visited:
                        visited.add(nbr)
                        dq.append((nbr, depth + 1))
        
        if not all_pos:
            #print('none')
            return []
        
        all_pos.sort(key=lambda p: self.dis_to_searched(p), reverse=True)
        #print('success bfs')
        return all_pos
    
    def dis_to_searched(self, point: shapely.Point):
        if not self.searched_points:
            return 1000
        
        min_dis = float('inf')
        pointkey = (round(point.x, 1), round(point.y, 1))
        
        for searched_key in self.searched_points:
            searched_point = shapely.Point(searched_key[0], searched_key[1])
            dis = point.distance(searched_point)
            if dis < min_dis:
                min_dis = dis
        
        #print('dsi')
        return min_dis
    
    def get_possible_hiderps(self, state: WorldState):
        if not self.last_seen:
            return self.map.random_position()
        
        all_pos = self.bfs(state)
        if all_pos:
            #print('i got here')
            return all_pos[0]
        
        return self.last_seen[-1]
    
    def act(self, state: WorldState) -> tuple[float, float] | None:
        self.cnt += 1
        
        self.see_hider(state)
        
        if self.target is None or state.seeker_position.distance(self.target) < 10:
            self.target = self.decide(state)
            self.last_hider_check = None

        self.prevposition = state.seeker_position
        currc = self.map.find_cell(state.seeker_position)
        target = self.target
        tarc = self.map.find_cell(target)
        
        if state.seeker_position == target or not self.map.in_bounds(target):
            self.target = None
            return
        
        if self.last_hider_check is None or self.cnt - self.last_hider_check > 5:
            self.last_hider_check = self.cnt
        
        if currc == tarc or self.map.has_line_of_sight(state.seeker_position, target):
            self.target = None
            dx, dy = (target.x - state.seeker_position.x, target.y - state.seeker_position.y)
            distance = math.dist((dx, dy), (0, 0))
            speed = min(distance, self.max_speed*0.9999999)
            dx, dy = speed * dx / distance, speed * dy / distance
            #print('act')
            return (dx, dy)
        
        if self.opq == [] and currc != tarc:
            tmp = self.astar(self, currc, tarc)
            if tmp == None:
                return (0, 0)
            self.opq = tmp
        cnt = 0
        while len(self.opq) > cnt:
            if self.map.has_line_of_sight(state.seeker_position, self.opq[cnt].polygon.centroid):
                cnt += 1
            else:
                break
        cnt -= 1
        if state.seeker_position == target or not self.map.in_bounds(target):
            self.opq = []
            return
        
        
        if cnt > 0:
            for i in range(0, cnt):
                self.opq.pop(0)
        
        if not self.map.has_line_of_sight(state.seeker_position, self.opq[0].polygon.centroid):
            self.opq.pop(0)
            #print('1')
            return
        if self.opq:
            dx, dy = (self.opq[0].polygon.centroid.x - state.seeker_position.x, 
                     self.opq[0].polygon.centroid.y - state.seeker_position.y)
        else:
            dx, dy = (target.x - state.seeker_position.x, target.y - state.seeker_position.y)
        
        
        
        dis = math.dist((dx, dy), (0, 0))
        spd = min(dis, self.max_speed*0.999999)
        
        if dis > 0:
            dx, dy = spd * dx / dis, spd * dy / dis
        else:
            dx, dy = 0, 0
            if self.opq:
                self.opq.pop(0)
        
        #print('act2')
        return (dx, dy)

    @property
    def is_seeker(self) -> bool:
        return True