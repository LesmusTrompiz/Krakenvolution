import rclpy
from rclpy.node import Node
from uahrk_scenario_msgs.msg import Scenario,Plato
from geometry_msgs.msg import Pose2D

CANASTA_AZUL = (
    0.225,
    3.000,
    90
)

CANASTA_VERDE = (
    1.775,
    3.000,
    90
)

MANTEL = True
PLATO = False


PLATO_AZUL1 = (
    0.225,
    2.775,
    90,
    MANTEL
)

PLATO_AZUL2 = (
    0.225,
    1.175,
    90,
    PLATO
)

PLATO_AZUL3 = (
    0.725,
    0.225,
    90,
    PLATO
)

PLATO_AZUL4 = (
    1.775,
    0.225,
    90,
    PLATO
)

PLATO_AZUL5 = (
    1.775,
    1.925,
    90,
    PLATO
)

PLATOS_AZULES = [
    PLATO_AZUL1,
    PLATO_AZUL2,
    PLATO_AZUL3,
    PLATO_AZUL4,
    PLATO_AZUL5
]

PLATO_VERDE1 = (
    1.775,
    2.775,
    90,
    MANTEL
)

PLATO_VERDE2 = (
    1.175,
    1.175,
    90,
    PLATO
)

PLATO_VERDE3 = (
    1.275,
    0.225,
    90,
    PLATO
)

PLATO_VERDE4 = (
    0.225,
    0.225,
    90,
    PLATO
)

PLATO_VERDE5 = (
    0.225,
    1.925,
    90,
    PLATO
)

PLATOS_VERDES = [
    PLATO_VERDE1,
    PLATO_VERDE2,
    PLATO_VERDE3,
    PLATO_VERDE4,
    PLATO_VERDE5
]

TARTA1 = (
    0.225,
    2.275,
    0
)

TARTA2 = (
    0.225,
    2.075,
    0
)

TARTA3 = (
    0.725,
    1.875,
    0
)

TARTA4 = (
    0.725,
    1.125,
    0
)

TARTA5 = (
    0.225,
    0.775,
    0
)

TARTA6 = (
    0.225,
    0.575,
    0
)


############
TARTA7 = (
    1.775,
    2.275,
    0
)

TARTA8 = (
    1.775,
    2.075,
    0
)

TARTA9 = (
    1.275,
    1.875,
    0
)

TARTA10 = (
    1.275,
    1.125,
    0
)

TARTA11 = (
    1.775,
    0.775,
    0
)

TARTA12 = (
    1.775,
    0.575,
    0
)

TARTAS = [TARTA1, TARTA2, TARTA3, TARTA4, TARTA5, TARTA6, TARTA7, TARTA8, TARTA9, TARTA10, TARTA11, TARTA12]



def get_blue_camp_scenario():
    scen = Scenario()


    for tarta in TARTAS:
        p = Pose2D()
        p.x = tarta[0]
        p.y = tarta[1]
        p.theta = tarta[2]
        scen.tarta.append(p)

    for plato in PLATOS_AZULES:
        plato = Plato()
        plato.equipo = Plato.AMIGO
        plato.pose.x = plato[0]
        plato.pose.y = plato[1]
        plato.pose.theta = plato[2]
        plato.protegido = plato[3]
        scen.platos.append(plato)

    for plato in PLATOS_VERDES:
        plato = Plato()
        plato.equipo = Plato.ENEMIGO
        plato.pose.x = plato[0]
        plato.pose.y = plato[1]
        plato.pose.theta = plato[2]
        plato.protegido = plato[3]
        scen.platos.append(plato)

    can = Pose2D()
    can.x = CANASTA_AZUL[0]
    can.y = CANASTA_AZUL[1]
    can.theta = CANASTA_AZUL[2]
    scen.canasta.append(can)
    return scen

def get_green_camp_scenario():
    scen = Scenario()


    for tarta in TARTAS:
        p = Pose2D()
        p.x = tarta[0]
        p.y = tarta[1]
        p.theta = tarta[2]
        scen.tarta.append(p)

    for plato in PLATOS_AZULES:
        plato = Plato()
        plato.equipo = Plato.ENEMIGO
        plato.pose.x = plato[0]
        plato.pose.y = plato[1]
        plato.pose.theta = plato[2]
        plato.protegido = plato[3]
        scen.platos.append(plato)

    for plato in PLATOS_VERDES:
        plato = Plato()
        plato.equipo = Plato.AMIGO
        plato.pose.x = plato[0]
        plato.pose.y = plato[1]
        plato.pose.theta = plato[2]
        plato.protegido = plato[3]
        scen.platos.append(plato)

    can = Pose2D()
    can.x = CANASTA_VERDE[0]
    can.y = CANASTA_VERDE[1]
    can.theta = CANASTA_VERDE[2]
    scen.canasta.append(can)
    return scen


class ScenarioNode(Node): 
    def __init__(self): 
        Node.__init__(self,'scenario')
        self.pub_scneario = self.create_publisher(Scenario, 'scenario', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.declare_parameter('/play_side', 'default')
        self.scenario_msg = Scenario()
        

    def timer_callback(self):
        # Look for robot TF
        side = self.get_parameter('/play_side').get_parameter_value().string_value

        if side == "blue":
            self.scenario_msg = get_blue_camp_scenario()
        elif side == "green":
            self.scenario_msg = get_green_camp_scenario()
        else: 
            self.scenario_msg = Scenario()
        
        # Update robot marker
        self.pub_scneario.publish(self.scenario_msg)


