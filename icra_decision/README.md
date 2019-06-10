# BT-Decision
Decision-making Module of ICRA 2019 RoboMaster

Default Role: Master!
Used for Wing TODO

1. change icra2019.launch
2. change the config.yaml

```
                     /---\
                    | ROS |
                    |Topic|
master􏰓􏰃􏰄􏰅􏰆􏰇􏰈􏰓􏰅􏰔􏰏􏰕􏰖􏰅􏰆􏰄􏰆􏰇􏰓􏰃􏰄􏰅􏰆􏰇􏰈􏰓􏰅􏰔􏰏􏰕􏰖􏰅􏰆􏰄􏰆-------/master/sync_state-------->wing
master􏰓􏰃􏰄􏰅􏰆􏰇􏰈􏰓􏰅􏰔􏰏􏰕􏰖􏰅􏰆􏰄􏰆􏰇􏰓􏰃􏰄􏰅􏰆􏰇􏰈􏰓􏰅􏰔􏰏􏰕􏰖􏰅􏰆􏰄􏰆<--------/wing/sync_state---------wing
                    |     |
                     \---/
```

Some Bugs:

1. SupplyAction Cooperation Failed with Auxiliary?
2. Supplying with Shoot?
3. Run with Shoot?
4. RunAwayEnemyField: fix the special num 1.75 -> 2.75. Solved.
5. How to Rotate when waiting buff?
6. What happend if some action failed? How to improve the robustness of the robots?
7. Problem of turn back action?
8. ShootActin Control: Start contrl and heat?