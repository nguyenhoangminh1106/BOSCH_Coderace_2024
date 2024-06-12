3. About the Challenge

3.1. Evaluation and Metrics
We have modified the CARLA Leaderboard source code specifically for this CodeRace competition. Our provided autonomous ego vehicle will drive through 10 different routes, filled with other NPC (Non-Player Character) vehicles. However, please note that we implemented our ego vehicle as a very simple path-tracking robot, with no consideration for obstacles. Therefore, it is expected that our ego vehicle will collide with other vehicles or static objects in the map layout, or even go out of its route lane. Such violations will incur penalties to the total score. Please refer to the official CARLA Leaderboard page for their scoring method.

Upon completion of the evaluation, you shall find in the coderace folder the recorded video, along with a stats.csv file for a summary of your statistics. (The file can be found at "CR24_STAR_R2/materials")

You may notice that we added a little twist here, by the score_over_time column. That's how we differentiate between the ones who play too safe and the ones who can adapt their ego vehicle to various situations. In other words, how fast you complete a route also counts.

Note: In case your ego vehicle fails to complete the route due to a collision, score_over_time will be computed as (score / route_timeout). Otherwise, you might accidentally get a higher score by crashing too soon.

Your final score will be an average of the score_over_time from each route: SUM(L2:L11)/10


3.2. Your Mission
Your mission, should you choose to accept it, is to improve our /workspace/team_code/drivers/baseline_driver.py module. Whatever code you add, we expect them to be within the /workspace/team_code/drivers directory, since you will submit this folder to us as your solution.

** Note **
- "codevehicle": The given source code
- "CR24_STAR_R2": Our optimized solution