<?php 
//echo $_SERVER['HTTP_USER_AGENT'];
$fArray  = file("//home/guest//targetVelocity.txt");
$len = sizeof($fArray);
$i = $len - 1;
echo $fArray[$i];
 ?>
