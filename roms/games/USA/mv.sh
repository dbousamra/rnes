for oldname in *
do
  newname=`echo $oldname | sed -e 's/ /_/g'`
  mv "$oldname" "$newname"
done
