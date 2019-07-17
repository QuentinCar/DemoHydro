
----------------------------------------------------------------------------
------- Instructions pour faire fonctionner le bateau automatique ----------
----------------------------------------------------------------------------
############################################################################
####################### Preparation du Bateau ##############################
############################################################################
1.
Brancher le Sonar au cable noir, le mettre en dessous du bateau (support prevu à cet effet) /!\ Attention au sens /!\ Si on a que des -1 en valeur renvoyée par le sonar alors il est dans le mauvais sens --> Le tourner de 180°

2.
Brancher la batterie, mettre l'interrupteur derrière la cabine sur On (sert à mettre en marche le moteur)

3.
Fermer le bateau à l'aide des vis. il y'en a 8 en tout

############################################################################
###################### Lancement de la mission #############################
############################################################################

1.
Se connecter avec le bureau à distance à la raspberry du bateau 
	SSID : raspi_derobat
	mdp : ENSTA_MerXXL
	IP : 10.3.141.1

2.
Sur la rasp
Lancer le programme ./Test_devices, il se trouve dans le dossier Documents/DeRoBat/Sonar 
Il sert à lire les données recu du sonar

3.
Sur la rasp, dans un autre cmd
Lancer la commande python3 Client.py, il se trouve dans le dossier Documents/DeRoBat/Communication 
Il sert à enoyer/recevoir les données

4.
Sur le pc 
Lancer le fichier ServerManu.py avec Spyder. On a un affichage de la bathymetrie (Scatter et contourf) et le retour de la caméra ou se situe le bateau

5.
La mission se lance automatiquement apres avoir definit les paramètres de calibration
