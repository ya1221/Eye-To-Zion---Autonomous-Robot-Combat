from influxdb_client_3 import InfluxDBClient3
import pandas as pd

class RobotDB:
    def __init__(self) -> None:
        self.token = "apiv3_CL-NZqu4HRpOYR7ENJmN3kwgRXCaFpNEEC_j0QHrBqwaMf-XKoFfWWbUq9Oh83xQKEZNGNI9PO44UPPTtr6OKQ"
        self.client = InfluxDBClient3(
            host="http://localhost:8086",
            token=self.token,
            org="Robot",
            database="sensors"
        )

    def get_recent_movement(self, seconds:int=10) -> pd.DataFrame:
        query:str = f"""
            SELECT * FROM robot_pose
            WHERE time > now() - INTERVAL '{seconds} seconds'
            ORDER BY time DESC 
            LIMIT 10
        """

        return self.client.query(query=query, language="sql").to_pandas()

    def get_recent_speed(self, seconds:int=10) -> pd.DataFrame:
        query:str = f"""
            SELECT * FROM robot_speed
            WHERE time > now() - INTERVAL '{seconds} seconds'
            ORDER BY time DESC
            LIMIT 10
        """

        return self.client(query=query, language="sql").to_pandas()




if __name__ == "__main__":
    db = RobotDB()
    df_pose:pd.DataFrame = db.get_recent_movement(seconds=5)
    print(df_pose)
    df_speed:pd.DataFrame = db.get_recent_speed(seconds=5)
    print(df_speed)

    
    
