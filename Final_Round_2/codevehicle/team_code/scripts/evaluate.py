from coderace_eval_runner import CodeRaceEvalRunner
import hydra
import traceback

import sys
sys.path.append("..")


def run_code_race(config):
    runner = CodeRaceEvalRunner(config, debug=True)
    try:
        runner.run()
    except Exception as e:
        print("> {}\033[0m\n".format(e))
        traceback.print_exc()
        runner.close()
    finally:
        runner.close()


@hydra.main(config_path="../configs", config_name="config.yaml")
def main(config):
    if config.route.name == "coderace":
        run_code_race(config)
    if config.route.name == "leaderboard":
        raise RuntimeError("Leaderboard evaluation not yet supported")


if __name__ == "__main__":
    main()
