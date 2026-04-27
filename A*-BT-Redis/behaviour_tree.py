import time

import py_trees

from A_alg import MAX_DISTANCE

#######################
# attack branch
class IsEnemyClose(py_trees.behaviour.Behaviour):
    def __init__(self, name = "is_enemy_close"):
        super(IsEnemyClose, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="dist_to_closest_enemy", access=py_trees.common.Access.READ)

    def update(self):
        dist_to_closest_enemy = 1.2 #self.blackboard.dist_to_closest_enemy

        if dist_to_closest_enemy is not None and dist_to_closest_enemy <= MAX_DISTANCE:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
    
class HasLineOfSightToEnemy(py_trees.behaviour.Behaviour):
    def __init__(self, name = "has_line_of_sight_to_enemy"):
        super(HasLineOfSightToEnemy, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="has_line_of_sight_to_closest_enemy", access=py_trees.common.Access.READ)

    def update(self):
        has_line_of_sight_to_closest_enemy = True #self.blackboard.has_line_of_sight_to_closest_enemy

        if has_line_of_sight_to_closest_enemy is not None and has_line_of_sight_to_closest_enemy:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
    
class AttackAction(py_trees.behaviour.Behaviour):
    def __init__(self, name = "attack_action"):
        super(AttackAction, self).__init__(name)

    def update(self):
        pass

#######################
# survival branch
class IsLowHealthOrOutnumbered(py_trees.behaviour.Behaviour):
    def __init__(self, name = "is_low_health_or_outnumbered"):
        super(IsLowHealthOrOutnumbered, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="health", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="num_enemies", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="num_teammates", access=py_trees.common.Access.READ)

    def update(self):
        health = 0.3 #self.blackboard.health
        num_enemies = 3 #self.blackboard.num_enemies
        num_teammates = 1 #self.blackboard.num_teammates

        if (health is not None and health < 0.5) or (num_enemies is not None and num_teammates is not None and num_enemies > num_teammates):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class AreTeammatesComing(py_trees.behaviour.Behaviour):
    def __init__(self, name="are_teammates_coming"):
        super(AreTeammatesComing, self).__init__(name)
        
    def update(self):
        return py_trees.common.Status.FAILURE # נניח שכרגע אף אחד לא בא

class HideAndWaitAction(py_trees.behaviour.Behaviour):
    def __init__(self, name="hide_and_wait_action"):
        super(HideAndWaitAction, self).__init__(name)

    def update(self):
        # כאן ניתן פקודה ל-A* למצוא מחסה
        return py_trees.common.Status.RUNNING

class RunFromEnemyAction(py_trees.behaviour.Behaviour):
    def __init__(self, name = "run_from_enemy_action"):
        super(RunFromEnemyAction, self).__init__(name)

    def update(self):
        # request backup if enemies are tied with teammates
        pass


survival_branch = py_trees.composites.Sequence(name="survival_branch", memory=False)

tactical_reaction_selector = py_trees.composites.Selector(name="tactical_reaction_selector", memory=False)

wait_for_teammates_branch = py_trees.composites.Sequence(name="wait_for_teammates_branch", memory=False)

wait_for_teammates_branch.add_children([
    AreTeammatesComing(name="teammates coming?"),
    HideAndWaitAction(name="hide and wait")
])

tactical_reaction_selector.add_children([
    wait_for_teammates_branch,               
    RunFromEnemyAction(name="run!")
])

survival_branch.add_children([
    IsLowHealthOrOutnumbered(name="in danger?"), # שומר הסף - התנאי שמתחיל את הכל
    tactical_reaction_selector                   # אם התנאי עבר, הפעל את בורר התוכניות
])

#######################
# help teammate branch
class IsTeammateInDanger(py_trees.behaviour.Behaviour):
    def __init__(self, name = "is_teammate_in_danger"):
        super(IsTeammateInDanger, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key = "requests_teammate_help", access = py_trees.common.Access.READ)
    
    def update(self):
        pass

class IsDistanceToTeammateAcceptable(py_trees.behaviour.Behaviour):
    def __init__(self, name = "is_distance_to_teammate_acceptable"):
        super(IsDistanceToTeammateAcceptable, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="dist_to_closest_teammate", access=py_trees.common.Access.READ)

    def update(self):
        pass

class DriveToTeammateAction(py_trees.behaviour.Behaviour):
    def __init__(self, name = "drive_to_teammate_action"):
        super(DriveToTeammateAction, self).__init__(name)

    def update(self):
        pass

#########################
# patrol branch
class MapsToGoalAction(py_trees.behaviour.Behaviour):
    def __init__(self, name = "maps_to_goal_action"):
        super(MapsToGoalAction, self).__init__(name)

    def update(self):
        pass


attack_branch = py_trees.composites.Sequence(name="attack_branch", memory = False)
attack_branch.add_children(
    [IsEnemyClose(name = "is enemy close"), 
     HasLineOfSightToEnemy(name = "has line of sight to enemy"),
     AttackAction(name = "attack action")])

help_teammate_branch = py_trees.composites.Sequence(name="help_teammate_branch", memory = False)
help_teammate_branch.add_children(
    [IsTeammateInDanger(name = "is teammate in danger"), 
     IsDistanceToTeammateAcceptable(name = "is distance to teammate acceptable"), 
     DriveToTeammateAction(name = "drive to teammate action")])

patrol_branch = py_trees.composites.Sequence(name="patrol_branch", memory = False)
patrol_branch.add_children(
    [MapsToGoalAction(name = "maps to goal action")])

root = py_trees.composites.Selector(name="root", memory = False)
root.add_children([survival_branch, attack_branch, help_teammate_branch, patrol_branch])

root.setup_with_descendants()
print(py_trees.display.unicode_tree(root))

while True:
    root.tick_once()
    
    time.sleep(0.1)