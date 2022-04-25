from pydantic import BaseSettings, Field


class Settings(BaseSettings):
    IMAGE_FILETYPE: str = Field(default="jpg")
    THERMAL_IMAGE_FILETYPE: str = Field(default="png")

    ROS_BRIDGE_HOST: str = Field(default="localhost")
    ROS_BRIDGE_PORT: int = Field(default=9090)

    STORAGE_FOLDER: str = Field(default="./temp_results")
    PUBLISHING_TIMEOUT: int = Field(default=30)
    INSPECTION_POSE_TIMEOUT: int = Field(default=60)
    GET_IMAGE_TIMEOUT: int = Field(default=5)

    MQTT_HOST: str = Field(default="localhost")
    MQTT_PORT: int = Field(default=1883)
    MQTT_USERNAME: str = Field(default="mosquitto")
    # The MQTT password should be set as an environment variable
    # "ISAR_TURTLEBOT_MQTT_PASSWORD" in settings.env

    class Config:
        env_prefix = "ISAR_TURTLEBOT_"


settings = Settings()
