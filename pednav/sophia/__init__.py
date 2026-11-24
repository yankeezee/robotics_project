"""Sophia's additive, Webots-independent navigation and analysis interface.

Depends only on the shared ``pednav.core`` module. Does not own or modify
Anastasiia's Webots world, scenario generator, reactive policy, or runner.
"""
from .interface import NavigationObservation, PlannerOutput
from .estimation import OnlinePedestrianVelocityEstimator
from .controller import PlannerAdapter

__all__ = ['NavigationObservation', 'PlannerOutput',
           'OnlinePedestrianVelocityEstimator', 'PlannerAdapter']
