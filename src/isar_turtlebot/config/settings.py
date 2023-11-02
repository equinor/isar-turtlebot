import importlib.resources as pkg_resources

from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    def __init__(self) -> None:
        try:
            with pkg_resources.path(f"isar_turtlebot.config", "settings.env") as path:
                env_file_path = path
        except ModuleNotFoundError:
            env_file_path = None
        super().__init__(_env_file=env_file_path)

    IMAGE_FILETYPE: str = Field(default="jpg")
    THERMAL_IMAGE_FILETYPE: str = Field(default="png")

    ROS_BRIDGE_HOST: str = Field(default="localhost")
    ROS_BRIDGE_PORT: int = Field(default=9090)

    TURTLEBOT_MAP: str = Field(default="klab_turtlebot")

    STORAGE_FOLDER: str = Field(default="./temp_results")
    PUBLISHING_TIMEOUT: int = Field(default=30)
    INSPECTION_POSE_TIMEOUT: int = Field(default=60)
    GET_IMAGE_TIMEOUT: int = Field(default=5)

    LOGGER_NAME: str = Field(default="isar_turtlebot")

    model_config = SettingsConfigDict(
        env_prefix="ISAR_TURTLEBOT_",
        env_file_encoding="utf-8",
        case_sensitive=True,
    )


settings = Settings()
