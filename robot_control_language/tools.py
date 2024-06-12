import json
from enum import Enum


class TemperatureUnit(Enum):
    CELSIUS = "celsius"
    FAHRENHEIT = "fahrenheit"


def get_current_weather(location: str, unit: str):
    """Get the current weather in a given location"""
    if "tokyo" in location.lower():
        return json.dumps({"location": "Tokyo", "temperature": "10", "unit": "celsius"})
    elif "san francisco" in location.lower():
        return json.dumps({"location": "San Francisco", "temperature": "72", "unit": "fahrenheit"})
    elif "paris" in location.lower():
        return json.dumps({"location": "Paris", "temperature": "22", "unit": "celsius"})
    else:
        return json.dumps({"location": location, "temperature": "unknown"})


def get_n_day_weather_forecast(location: str, format: TemperatureUnit, num_days: int):
    return json.dumps({"location": location, "temperature": "25", "unit": "celsius", "num_days": "5"})


hercules_functions = {
    "get_current_weather": get_current_weather,
    "get_n_day_weather_forecast": get_n_day_weather_forecast,
}