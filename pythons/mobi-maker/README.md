# mycover.jpg
This is the cover of your book.
# mykindlebook.html
This is the content of your book — what your readers will read. 

Keep the HTML simple and refer to Amazon’s documentation for details on how they want you to write your markup.
# toc.html
This is the table of contents.

You’ll see that each link points to a part of the Kindle book using anchor points.
# style.css
This is your stylesheet and it’s how you’ll design your book using CSS 3.

I’ve styled my book so text is green (cripes) and images float to the right.

Amazon’s support for CSS 3 is relativelu new and pretty darn exciting because of the new design options it brings.

You can learn more about the CSS 3 tags Amazon supports on its various devices here.
# mykindlebook.opf
This is an XML file that tells the Kindle how your book is structured.

Go ahead and open it up (you can use TextEdit if you don’t have a code editor).

The stuff between the <metadata> tags defines the metadata for the book using standard Dublin Core elements:
title, creator, date, etc.

The stuff between the <manifest> tags tells Kindle where things are.    
For example, you’ll see that it has references to all the files we’re currently reviewing:    
the cover image, the book HTML file, the table of contents HTML file, the stylesheet,    
and the NCX file (I’ll explain that in a moment).

The stuff between the <spine> tags tells Kindle the order of how those HTML files should be read.   
Using this file, for example, when you open your book on a Kindle you’ll first hit the table of contents.   
Then you’ll hit the book contents.    
See how the “idref” in the <spine> elements match the “id” in the <manifest> elements?   
That’s how the spine knows what HTML files to present.    
That matching is required, of course, otherwise Kindle won’t know what to load.    
You can create multiple HTML files and define the order they will be accessed in the <spine>.    
Be sure to also make reference to those HTML files in your <manifest> so that Kindle knows where they are located!
# toc.ncx
You know that bar along the bottom of a Kindle that has points that allow you to skip to various parts?    
The NCX file tells Kindle where those points are. The <navPoint> elements within the <navMap> tags define those points.

I also put a folder called “img” in the download and I threw in a nice de Kooning
picture that you’ll see embedded in mykindlebook.html file.

And that’s it. These are the minimum files you need to build a Kindle book.