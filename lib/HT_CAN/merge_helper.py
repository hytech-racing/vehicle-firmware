# this script helps with loading and merging dbc files with hytech's pcan symbol file
import cantools
import cantools.database
base_db = cantools.database.load_file('PCAN_project/hytech.sym')
base_db.add_dbc_file('imported_dbcs/SpeedBeam_bus1.dbc')



cantools.database.dump_file(base_db, 'PCAN_project/hytech.sym')