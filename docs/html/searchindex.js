Search.setIndex({"docnames": ["creating_plugin", "index", "introduction", "user_guide", "user_guide/counter_details", "user_guide/counter_use", "user_guide/scanner_details", "user_guide/scanner_use"], "filenames": ["creating_plugin.rst", "index.rst", "introduction.rst", "user_guide.rst", "user_guide/counter_details.rst", "user_guide/counter_use.rst", "user_guide/scanner_details.rst", "user_guide/scanner_use.rst"], "titles": ["Creating a plugin for your specific needs", "Welcome to pymodaq_plugins_daqmx\u2019s documentation!", "Introduction", "User guide", "Details about the implementation of the OD viewer PLcounter", "Using the 0D viewer PLcounter", "Details about the implementation of the MultipleScannerControl move", "Using the ScannerControl and MultipleScannerControl moves"], "terms": {"In": [0, 1, 6], "order": [0, 6], "us": [0, 1, 2, 3, 4], "ni": [0, 1, 2, 3, 5, 6, 7], "card": [0, 1, 2, 5, 6, 7], "acquir": [0, 2], "some": 0, "data": [0, 5], "control": [0, 3, 4, 7], "an": [0, 3, 4, 5, 6, 7], "actuat": [0, 1, 2, 6], "even": [0, 3, 7], "synchron": 0, "acquisit": [0, 1, 2, 4, 5], "anoth": [0, 5, 6, 7], "devic": [0, 1, 2, 3, 5, 6, 7], "you": [0, 1, 2, 3, 4, 5, 6, 7], "should": [0, 1, 7], "start": [0, 5, 6], "usual": [0, 3, 5, 6], "from": [0, 2, 4, 5, 6, 7], "templat": 0, "It": [0, 1, 3, 5, 7], "i": [0, 1, 2, 3, 4, 5, 6, 7], "also": [0, 1, 2, 6, 7], "strongli": 0, "recommend": [0, 5, 7], "read": [0, 7], "tutori": [0, 1], "about": [0, 3, 5, 7], "instrument": [0, 1, 2], "avail": [0, 3, 7], "pymodaq": [0, 1, 2, 6], "document": [0, 5], "Then": [0, 2], "have": [0, 2, 3, 4, 6], "analyz": 0, "experi": 0, "want": [0, 6], "perform": [0, 1, 2, 4, 6, 7], "find": 0, "best": 0, "type": [0, 3, 7], "singl": [0, 1, 3, 4, 5, 6, 7], "daqmx": [0, 3, 4, 6], "object": [0, 1, 3, 4], "onli": [0, 3, 5, 6], "one": [0, 2, 3, 4, 5, 6, 7], "task": [0, 2, 3, 4, 5, 6, 7], "dict": [0, 3, 4, 6], "sever": [0, 3, 6, 7], "like": [0, 5, 7], "plcounter": [0, 3, 6], "exampl": [0, 1, 3], "combin": [0, 3, 7], "other": [0, 3, 4, 5, 6, 7], "hardwar": [0, 2, 3, 4, 5, 6], "synchronis": [0, 6], "which": [0, 1, 2, 3, 4, 5, 6, 7], "trigger": 0, "more": [0, 3], "complic": [0, 3], "write": 0, "multiplescannercontrol": [0, 3], "share": [0, 6, 7], "resourc": [0, 5, 7], "between": [0, 2, 7], "differ": [0, 6, 7], "independ": 0, "paramet": [0, 4, 5, 6, 7], "measur": [0, 4, 5], "bewar": [0, 3], "particular": 0, "scan": [0, 6, 7], "extens": [0, 6, 7], "send": [0, 5, 6], "all": [0, 6], "move": [0, 1, 3], "command": [0, 6], "same": [0, 5, 6, 7], "time": [0, 2, 4, 5, 7], "thi": [1, 3, 4, 5, 6, 7], "plugin": [1, 2, 4, 5, 6, 7], "devot": 1, "nation": 1, "signal": [1, 2, 5, 6], "gener": [1, 2], "nidaqmx": [1, 2], "librari": 1, "contain": [1, 3, 4, 6], "basi": 1, "build": [1, 2, 3], "viewer": [1, 3], "custom": 1, "addit": [1, 6], "thei": [1, 2, 3], "ar": [1, 2, 3, 4, 6, 7], "meant": [1, 5, 7], "provid": [1, 2, 5], "can": [1, 2, 3, 5, 6, 7], "piezo": [1, 6], "scanner": [1, 3, 7], "analog": [1, 2, 3, 6, 7], "output": [1, 3, 6, 7], "function": [1, 3, 4], "well": 1, "0d": [1, 3], "correspond": [1, 5, 6], "photon": [1, 3, 5], "counter": [1, 3, 4, 5, 6, 7], "written": 1, "guid": 1, "when": [1, 6, 7], "set": [1, 4, 6, 7], "up": [1, 4], "your": [1, 2, 3, 5, 6, 7], "compat": 1, "4": [1, 5], "introduct": 1, "user": 1, "creat": [1, 3, 7], "specif": [1, 3, 7], "need": [1, 2, 3, 5, 6, 7], "veri": [2, 6, 7], "versatil": 2, "both": [2, 4], "detector": [2, 5], "offer": 2, "possibl": [2, 3], "digit": [2, 3], "extern": 2, "intern": [2, 5], "As": [2, 3, 4, 6, 7], "consequ": [2, 3, 6], "hardli": 2, "oper": [2, 7], "most": [2, 3, 6, 7], "probabl": [2, 3, 6, 7], "own": [2, 3], "modul": 2, "precis": 2, "mind": 2, "Of": 2, "cours": 2, "work": [2, 5, 6, 7], "see": [2, 3, 4, 7], "here": [2, 4, 6], "procedur": [2, 3], "driver": 2, "comput": 2, "togeth": [2, 4, 6], "max": 2, "softwar": 2, "test": [2, 5, 7], "pymodaq_plugins_daqmx": 2, "manag": 2, "pip": 2, "The": [2, 3, 4, 5, 6, 7], "interfac": 2, "python": 2, "thu": [2, 6, 7], "done": [2, 6], "packag": 2, "pydaqmx": 2, "now": 3, "instal": 3, "two": [3, 4, 7], "option": 3, "either": 3, "alreadi": 3, "do": [3, 5, 7], "exactli": 3, "what": [3, 6], "daq_mov": 3, "daq_view": 3, "count": [3, 4, 5], "daqmx_pl_count": 3, "piezoelectr": [3, 7], "desir": 3, "slow": [3, 7], "speed": [3, 7], "daqmx_scannercontrol": 3, "daqmx_multiplescannercontrol": 3, "first": [3, 4, 7], "explain": 3, "how": [3, 5, 7], "detail": 3, "scannercontrol": [3, 6], "For": [3, 4, 6], "ani": [3, 6, 7], "base": 3, "wrapper": 3, "alwai": 3, "These": [3, 4, 7], "defin": [3, 6], "national_instru": 3, "py": [3, 4, 6], "Such": 3, "configur": [3, 4], "sent": [3, 4, 5, 6, 7], "channel": [3, 4, 5, 6, 7], "input": [3, 5, 7], "them": [3, 4, 6, 7], "bit": 3, "might": [3, 5, 6, 7], "handl": [3, 4], "situat": 3, "given": [3, 5], "follow": 3, "section": 3, "implement": 3, "od": 3, "seen": 4, "we": [4, 6], "second": 4, "2": [4, 6], "separ": 4, "although": 4, "connect": [4, 6, 7], "result": [4, 6], "cannot": [4, 5, 6, 7], "clock": [4, 5, 6, 7], "each": [4, 5, 6, 7], "clockcount": 4, "file": [4, 6], "its": [4, 6], "definit": [4, 5], "sourc": [4, 5], "contin": 4, "grab": [4, 5], "snap": [4, 5], "By": 4, "default": [4, 5], "grab_data": 4, "reset": 4, "issu": [4, 6], "continu": [4, 5], "doe": 4, "make": [4, 6, 7], "sens": 4, "so": [4, 6, 7], "live": 4, "decid": 4, "updat": 4, "consist": 5, "ttl": 5, "puls": 5, "mainli": 5, "avalanch": 5, "photodiod": 5, "apd": 5, "displai": [5, 6], "kcount": 5, "": [5, 7], "wa": [5, 6, 7], "pcie": 5, "63xx": 5, "shown": [5, 7], "screenshot": [5, 7], "below": [5, 7], "rise": 5, "edg": 5, "someth": [5, 7], "dev": [5, 7], "x": [5, 6, 7], "ctr": [5, 7], "y": [5, 6, 7], "pfi": 5, "choos": 5, "list": [5, 6, 7], "frequenc": 5, "rate": 5, "point": 5, "dure": [5, 6, 7], "associ": 5, "interv": 5, "intend": 5, "slave": 5, "Be": [5, 7], "care": [5, 6, 7], "termin": 5, "check": 5, "befor": [5, 6, 7], "launch": 5, "If": [5, 6, 7], "forget": [5, 7], "stop": [5, 7], "otherwis": [5, 7], "get": [5, 6, 7], "error": [5, 7], "being": [5, 7], "busi": [5, 7], "mani": [5, 6, 7], "warn": [5, 7], "log": [5, 7], "finish": [5, 7], "worri": [5, 7], "still": [5, 7], "fine": [5, 7], "know": [5, 6, 7], "solv": [5, 7], "pleas": [5, 7], "contribut": [5, 7], "similarli": 6, "case": 6, "principl": 6, "trick": 6, "mean": [6, 7], "quickli": 6, "becom": 6, "movement": [6, 7], "everi": 6, "roughli": 6, "simultan": 6, "troubl": 6, "solut": 6, "propos": 6, "add": 6, "ao_with_clock_daqmx": 6, "daqmx_object": 6, "instead": 6, "ha": 6, "attribut": 6, "modifi": 6, "master": 6, "analogoutput": 6, "store": 6, "aochannel": 6, "describ": 6, "reformat": 6, "valu": [6, 7], "exempl": 6, "1": [6, 7], "5": 6, "\u00b5m": 6, "step": [6, 7], "1000": 6, "2000": 6, "3000": 6, "4000": 6, "5000": 6, "current_y_posit": 6, "allow": [6, 7], "after": 6, "without": 6, "conflict": 6, "achiev": 6, "lock": 6, "true": 6, "long": 6, "variabl": 6, "wait": [6, 7], "state": 6, "waiting_to_mov": 6, "until": 6, "ni_card_ready_for_mov": 6, "receiv": 6, "indic": [6, 7], "resum": 6, "over": [6, 7], "move_done_sign": 6, "emit": 6, "current": 6, "close": 6, "enough": [6, 7], "switch": 6, "unlock": 6, "back": [6, 7], "fals": 6, "pilot": 6, "voltag": [6, 7], "appli": 6, "index": 6, "nm": [6, 7], "restart": [6, 7], "caus": 6, "quick": 6, "go": 6, "zero": 6, "made": 7, "attocub": 7, "amplifi": 7, "take": 7, "main": 7, "reliabl": 7, "therefor": 7, "page": 7, "focuss": 7, "latter": 7, "posit": 7, "primari": 7, "xy": 7, "atom": 7, "forc": 7, "microscop": 7, "quit": 7, "purpos": 7, "avoid": 7, "damag": 7, "tip": 7, "fast": 7, "specifi": 7, "ao": 7, "size": 7, "m": 7, "convers": 7, "factor": 7, "v": 7, "coeffici": 7, "convert": 7, "displac": 7, "statu": 7, "treat": 7, "timeout": 7, "larg": 7, "bound": 7, "rang": 7, "reachabl": 7, "small": 7, "number": 7, "initi": 7, "ident": 7, "except": 7, "taken": 7, "proper": 7, "id": 7, "regular": 7, "daq": 7, "notic": 7, "never": 7, "goe": 7, "0": 7, "stai": 7, "becaus": 7, "wai": 7, "mention": 7, "ask": 7, "chang": 7, "larger": 7, "than": 7, "durat": 7, "relat": 7, "pixel": 7, "click": 7, "red": 7, "squar": 7, "look": 7}, "objects": {}, "objtypes": {}, "objnames": {}, "titleterms": {"creat": 0, "plugin": [0, 3], "your": 0, "specif": 0, "need": 0, "welcom": 1, "pymodaq_plugins_daqmx": 1, "": 1, "document": 1, "content": 1, "introduct": [2, 6], "instal": 2, "user": 3, "guid": 3, "work": 3, "principl": 3, "detail": [4, 6], "about": [4, 6], "implement": [4, 6], "od": 4, "viewer": [4, 5], "plcounter": [4, 5], "us": [5, 6, 7], "0d": 5, "configur": [5, 7], "multiplescannercontrol": [6, 7], "move": [6, 7], "new": 6, "object": 6, "control": 6, "handl": 6, "time": 6, "read": 6, "scanner": 6, "posit": 6, "scannercontrol": 7, "master": 7, "slave": 7}, "envversion": {"sphinx.domains.c": 3, "sphinx.domains.changeset": 1, "sphinx.domains.citation": 1, "sphinx.domains.cpp": 9, "sphinx.domains.index": 1, "sphinx.domains.javascript": 3, "sphinx.domains.math": 2, "sphinx.domains.python": 4, "sphinx.domains.rst": 2, "sphinx.domains.std": 2, "sphinx": 58}, "alltitles": {"Creating a plugin for your specific needs": [[0, "creating-a-plugin-for-your-specific-needs"]], "Welcome to pymodaq_plugins_daqmx\u2019s documentation!": [[1, "welcome-to-pymodaq-plugins-daqmx-s-documentation"]], "Contents:": [[1, null]], "Introduction": [[2, "introduction"]], "Installation": [[2, "installation"]], "User guide": [[3, "user-guide"]], "Working principle of the plugins": [[3, "working-principle-of-the-plugins"]], "Details about the implementation of the OD viewer PLcounter": [[4, "details-about-the-implementation-of-the-od-viewer-plcounter"]], "Using the 0D viewer PLcounter": [[5, "using-the-0d-viewer-plcounter"]], "Configuration": [[5, "configuration"], [7, "configuration"]], "Use": [[5, "use"], [7, "use"]], "Details about the implementation of the MultipleScannerControl move": [[6, "details-about-the-implementation-of-the-multiplescannercontrol-move"]], "Introduction of a new object to use as controller": [[6, "introduction-of-a-new-object-to-use-as-controller"]], "Handling of the timing": [[6, "handling-of-the-timing"]], "Reading the scanner position": [[6, "reading-the-scanner-position"]], "Using the ScannerControl and MultipleScannerControl moves": [[7, "using-the-scannercontrol-and-multiplescannercontrol-moves"]], "Master": [[7, "master"]], "Slave": [[7, "slave"]]}, "indexentries": {}})