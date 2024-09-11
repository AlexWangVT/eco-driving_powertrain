# install.packages('RSQLite')

library('RSQLite')

# connect to the sqlite file
sqlite    <- dbDriver("SQLite")
exampledb <- dbConnect(sqlite,"FM528-Aimsun1.sqlite")

dbListTables(exampledb)
