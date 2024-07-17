# APP6 - Programmation concurrente (Été 2024)

Ce dépôt contient le code de départ pour l'APP6 du cours GRO300.

## Prérequis

- Connexion Internet
- Raspberry Pi 5 avec la dernière version de Raspberry Pi OS
- Une copie complète de ce dépôt (suivez les instructions sur GitHub pour le clôner ou télécharger la copie en format `zip`).

Notez que le code devrait être utilisable dans n'importe système Linux ou macOS récent. Or, vous devrez vous-mêmes adapter les instructions suivantes pour compiler et exécuter le code de cet APP dans l'environnement de votre choix.  

## Structure

 - `envsim.py` : Le point de départ Python du simulateur pour la problématique
  - `robot/` : Code de simulation de l'état d'un ensemble de robots.
  - `src/robotsim.cpp` : Sous-processus C++ appelé par la partie Python. 
  - `lab/ex*.cpp` : Exercices de la période de laboratoire

## Installation

1. Mettez à jour votre système :

```bash
sudo apt update
sudo apt upgrade
```

2. Dépendances nécessaires (certaines sont peut-être déjà installées) :

```bash
sudo apt install cmake build-essential python3-pip
```

3. Installation de Visual Studio Code (si ce n'est pas déjà fait) : 

```bash
sudo apt install code
```

On recommande également d'installer les extensions C++ et Python, incluant "Python Debugger".

4. Création de l'environnement virtuel Python :

Ouvrez le dossier sous Visual Studio Code et un terminal. Tapez ensuite :

```bash
python3 -m venv venv
source venv/bin/activate
```

5. Installation des dépendances Python dans l'environnement virtuel :

```bash
pip install -r requirements.txt
```

## Utilisation

Vous pouvez maintenant explorer et modifier le code. Trois configurations de lancement sont offertes :

### Python: Lancer envsim avec le sous-processus robotsim (C++)
Cette configuration permet de lancer le script Python `envsim.py`. Elle est configurée pour exécuter le programme dans le répertoire build pour avoir accès au sous-processus C++ compilé. Avant de lancer le script, elle exécute la tâche "CMake: construire" pour s'assurer que le code C++ est compilé. Cette configuration est utile pour déboguer l'ensemble du système, y compris l'interaction entre Python et le sous-processus C++.

### C++: Attacher au sous-processus
Cette configuration permet d'attacher le débogueur C++ à un processus en cours d'exécution. L'utilisateur peut choisir le processus auquel s'attacher. C'est particulièrement utile pour déboguer le sous-processus C++ robotsim une fois qu'il est lancé par le script Python. Il suffit d'abord de lancer `envsim` avec la procédure précédente. Le numéro de PID du sous-processu sera affiché à la console. Ensuite, il suffit de lancer cette configuration et spécifier le PID qui apparaît à la console. L'utilisateur peut choisir entre GDB et LLDB comme débogueur.

### C++: Lancer le fichier actuel
Cette configuration permet de lancer et déboguer le fichier C++ actuellement ouvert dans l'éditeur. Elle compile d'abord le projet avec la tâche "CMake: construire", puis lance le programme correspondant dans le dossier build. Cette configuration est utile pour tester et déboguer les exercices du laboratoire (fichiers contenus dans `lab/`).

## Compilation manuelle (alternative)

Si vous ne voulez pas utiliser Visual Studio Code, vous pouvez aussi manuellement
compiler le code, les requis sont les mêmes (cmake, python3-pip et les requis de
requirements.txt).

Depuis le dossier du projet :

```bash
mkdir build
cd build
cmake ../
cmake --build .
```

Vous retrouverez maintenant tous les exécutables C++ et vous pourrez lancer la
simulation ainsi :

```bash
python3 ../envsim.py
```