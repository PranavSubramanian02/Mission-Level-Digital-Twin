# Pydantic request and response contracts for FastAPI Digital Twin server

from typing import List, Optional
from pydantic import BaseModel, Field


class PreflightRequest(BaseModel):
    v_cruise_ms: float = Field(18.0, ge=12.0, le=30.0)
    v_loiter_ms: float = Field(14.0, ge=11.0, le=22.0)
    wind_north_ms: float = Field(-4.0, ge=-20.0, le=20.0)
    wind_east_ms: float = Field(1.5, ge=-20.0, le=20.0)
    gust_sigma_ms: float = Field(1.8, ge=0.0, le=8.0)
    r0_ohm: float = Field(0.085, ge=0.02, le=0.35)
    n_runs: int = Field(1000, ge=100, le=10000)
    include_rtb: bool = True


class ParetoRequest(BaseModel):
    wind_north_ms: float = Field(-4.0, ge=-20.0, le=20.0)
    wind_east_ms: float = Field(1.5, ge=-20.0, le=20.0)
    gust_sigma_ms: float = Field(1.8, ge=0.0, le=8.0)
    r0_ohm: float = Field(0.085, ge=0.02, le=0.35)
    n_runs_per_point: int = Field(400, ge=100, le=2000)


class TwinStartRequest(BaseModel):
    route_index: int = Field(0, ge=0, le=2)
    v_cruise_ms: float = Field(18.0, ge=12.0, le=30.0)
    wind_north_ms: float = Field(-4.0, ge=-20.0, le=20.0)
    wind_east_ms: float = Field(1.5, ge=-20.0, le=20.0)
    gust_sigma_ms: float = Field(1.8, ge=0.0, le=8.0)


class FaultInjectRequest(BaseModel):
    fault_type: str = Field(..., pattern="^(icing_drag|battery_aging|wind_shift)$")
    magnitude: float


class TwinStepRequest(BaseModel):
    dt_s: float = Field(5.0, ge=1.0, le=30.0)
    mc_runs: int = Field(300, ge=100, le=2000)


class ExportRequest(BaseModel):
    dpi: int = Field(300, ge=100, le=600)
    n_runs: int = Field(1000, ge=200, le=5000)
    output_dir: Optional[str] = None
