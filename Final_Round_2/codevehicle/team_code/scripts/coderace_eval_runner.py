import numpy as np
from leaderboard.coderace_evaluator import (
    CodeRaceEvaluator
)


class Args:
    pass


class CodeRaceEvalRunner:
    def __init__(self, config, debug=False):
        args = Args()

        args.port = config.port
        args.agent = config.agent
        args.agent_config = config.driver.config_path
        args.scenario_class = config.route.scenario
        args.host = config.host
        args.output_path = config.output_path
        args.track = config.track
        args.timeout = config.carla_timeout
        tm_port = np.random.randint(10000, 60000)
        args.trafficManagerPort = tm_port
        args.trafficManagerSeed = 0
        args.debug = debug
        args.resume = False
        args.record = ""

        self.runner = CodeRaceEvaluator(args)
        self.args = args

    def close(self):
        if hasattr(self, "server") and self.server:
            self.server.close()

    def run(self):
        return self.runner.run(self.args)
