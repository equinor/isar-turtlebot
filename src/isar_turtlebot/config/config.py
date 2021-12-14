import importlib.resources as pkg_resources
from configparser import ConfigParser
from os import getenv

from dotenv import load_dotenv
from isar_turtlebot.config.configuration_error import ConfigurationError


class Config(object):
    def __init__(self):
        load_dotenv()

        self.parser = ConfigParser()

        with pkg_resources.path("isar_turtlebot.config", "default.ini") as filepath:
            found_default: bool = self.parser.read(filepath)

        if not found_default:
            raise ConfigurationError(
                f"Failed to import configuration, found default: {found_default}"
            )

    def get(self, section, option):
        return self.parser.get(section, option)

    def getint(self, section, option):
        return self.parser.getint(section, option)

    def getfloat(self, section, option):
        return self.parser.getfloat(section, option)

    def getbool(self, section, option):
        return self.parser.getboolean(section, option)

    def sections(self):
        return self.parser.sections()


config = Config().parser
