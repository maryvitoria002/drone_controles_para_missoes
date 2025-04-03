from geopy.distance import distance
from geopy.point import Point
from pymavlink import mavutil
from math import radians, sin, cos, sqrt, atan2, asin, degrees

class GPS: 
    def __init__(self) -> None:
        self.directions_dict = {
            'north': 0,
            'east': 90,
            'south': 180,
            'west': 270
        }
        
    @staticmethod
    def haversine(lat1, lon1, lat2, lon2):
        # Converter graus para radianos
        lat1 = radians(lat1)
        lon1 = radians(lon1)
        lat2 = radians(lat2)
        lon2 = radians(lon2)   

        # Diferença entre as coordenadas
        dlat = lat2 - lat1
        dlon = lon2 - lon1

        # Fórmula de Haversine
        a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))

        # Raio médio da Terra em metros
        R = 6371000  # metros

        # Distância em metros
        distance = R * c

        return distance

    def calculate_coord(self, lat_initital, long_inital, meters, direction):
        """
        Calcula a nova coordenada após deslocamento de uma distância em meters em uma direção específica.

        :param lat_initital: Latitude inicial
        :param long_inital: Longitude inicial
        :param meters: Distância em meters para deslocamento
        :param direction: Direção do deslocamento em graus (north, east, south, west)
        :return: (new_lat, new_long)
        """
       
        ponto_inicial = Point(lat_initital, long_inital)
        new_distance = distance(meters=meters)
        new_point = new_distance.destination(point=ponto_inicial, bearing=self.directions_dict[direction])
        
        new_lat = new_point.latitude
        new_long = new_point.longitude
        
        return new_lat, new_long

    
    def meters_to_geo(self, lat, lon, distance_m, bearing_deg):
        # Raio médio da Terra em metros
        R = 6367400  

        # Converter graus para radianos
        lat_rad = radians(lat)
        lon_rad = radians(lon)
        bearing_rad = radians(bearing_deg)

        # Calcular a nova latitude
        lat2_rad = asin(sin(lat_rad) * cos(distance_m / R) +
                         cos(lat_rad) * sin(distance_m / R) * cos(bearing_rad))

        # Calcular a nova longitude
        lon2_rad = lon_rad + atan2(sin(bearing_rad) * sin(distance_m / R) * cos(lat_rad),
                                    cos(distance_m / R) - sin(lat_rad) * sin(lat2_rad))

        # Converter de volta para graus
        lat2 = degrees(lat2_rad)
        lon2 = degrees(lon2_rad)

    # Normalizar a longitude para ficar entre -180 e 180 graus
        lon2 = (lon2 + 540) % 360 - 180

        return lat2, lon2
    
# # Exemplo de uso
# lat_initital = -14.303033
# long_inital = -42.6944275  
# meters = 6
# directions = 'east'  # Norte

# gps = GPS()
# new_lat, new_long = gps.calculate_coord(lat_initital, long_inital, meters, directions)
# print(f"Nova latitude: {new_lat}, Nova longitude: {new_long}")
