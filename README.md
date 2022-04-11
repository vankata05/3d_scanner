# 3d_scanner

Преносим 3д скенер работещ с Arduino nano 2040 connect 
който оперира с инфрачервен сензор за дистанция и няколко нема 17 мотора.


3D скенера функционира по следния начин:

  1)Измерва разтоянието от сензора до сканирания обест след което това разтояние бива използвано за изчислението на разтоянието от центъра до повърхността на обекта;
  
  2)Тъй като използваният сензор е аналогов се налага да изчистиме шума което постигам с измерване на всяка точка по 1000 пъти, след което смятам средноаритметично и записвам получината инфобмация в масив;
  
  3)След като успешно съм измерил дадена точка движа платформата с една стъпка и повтарям процеса докато платформата не се завърти на 360 градуса след което движа сензора нагоре, и повтарям това докато сензора не стигне до края на обекта;
  
  4)Накрая остава само да се запише информацията на SD под формата на pointcloud и да се трансформира в mesh.
  
  
Начин на употреба:
  
  1)Вкараите захранването в power jack който се намира в заднача част на куфара;
  
  
  2)Уверете се че куфара е отворен до край;
  
  
  3)Вкарайте SD карта;
  
  
  4)Поставете обекта който бихте желали да сканирате върху пратформата;
  
  
  5)В менюто откриите "Start scan";
  
  
  6)Натиснете бутона на енкодера за да стартирате сканирането;
  
  
  7)Изчакайте сканирането да приключи;
  
  
  8)Локирайте "Save to SD" в менюто и натиснете бутона;
  
  
  9)Извадете картата и на нея ще откриете генерирания MESH.TXT файл;
  
  
  10)Готови сте :-)
