<?php 
//echo $_SERVER['HTTP_USER_AGENT'];
$output = shell_exec('ls -l');
//echo "<pre>$output</pre>";
echo "<br/>";
if(strpos($output, 'running')!= FALSE)
{
echo 'running';
}
else
{
echo 'not Running';
} ?>
