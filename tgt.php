<?php 
//echo $_SERVER['HTTP_USER_AGENT'];
$fArray  = file("//home/guest//targetData.txt");
$len = sizeof($fArray);
$i = $len - 1;
echo $fArray[$i];
 ?>
