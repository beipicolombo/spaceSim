
import src.interfaces.eventsAndTmTc as eventsAndTmTc


# Predefined event database
EVT_DATABASE = eventsAndTmTc.EventDatabase()
EVT_DATABASE.addEvent(eventsAndTmTc.Event(name = "NULL" , id = -1))
EVT_DATABASE.addEvent(eventsAndTmTc.Event(name = "EVT_AOCS_MODE_SWITCH" , id = 1))
EVT_DATABASE.addEvent(eventsAndTmTc.Event(name = "EVT_TC_RECEIVED" , id = 2))
EVT_DATABASE.addEvent(eventsAndTmTc.Event(name = "EVT_TC_ACCEPTED" , id = 3))
EVT_DATABASE.addEvent(eventsAndTmTc.Event(name = "EVT_TC_REJECTED" , id = 4))

# Predefined TC database
TC_DATABASE = eventsAndTmTc.TcDatabase()
TC_DATABASE.addTc(eventsAndTmTc.Tc(name = "NULL", id = -1))
TC_DATABASE.addTc(eventsAndTmTc.Tc(name = "TC_AOCS_MODE_SWITCH_SAFE_TO_NOM_PTNG", id = 1))
TC_DATABASE.addTc(eventsAndTmTc.Tc(name = "TC_AOCS_MODE_SWITCH_NOM_PTNG_TO_NOM_EQ", id = 2))