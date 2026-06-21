from pydantic import BaseModel
from typing import List

class RobotPosition(BaseModel):
    x: float
    y: float
    delta: float = 0.5

class VectorPathRequest(BaseModel):
    start_x: float
    start_y: float
    end_x: float
    end_y: float

class GeoPoint(BaseModel):
    lat: float
    lon: float

class GeofencePolygon(BaseModel):
    points: List[GeoPoint]  # >= 3 points

class WaypointItem(BaseModel):
    id: int
    lat: float
    lon: float
    label: str = ""

class WaypointList(BaseModel):
    waypoints: List[WaypointItem]
    loop: bool = False

class SpeedRequest(BaseModel):
    multiplier: float = 1.0

class PlacementRequest(BaseModel):
    type: str   # 'agent' | 'target' | 'patrol' | 'aggressive'
    lat: float
    lon: float

class ScenarioSaveRequest(BaseModel):
    name: str
    params: dict = {}
