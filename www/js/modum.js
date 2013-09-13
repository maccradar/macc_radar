$(document).ready(function () {
	//last-child fix
	$('.main_menu li:last-child a, .footer-menu li:last-child a').addClass('last');
	
	//hide/show menu
	$('.main_menu ul ul').hide();
	$('.main_menu ul li.parent > ul').show();
	$('.main_menu ul li.active > ul').show();
	$('.main_menu ul li.active > ul > ul').hide();
	
	//font-resizer
	$('.font-resizer div').click(function(){
		var StdFontSize = 12;
		var largestFontSize = 18;
		var smallestFontSize = 10;
		
		var FontSizeValue = parseFloat($('p').css('fontSize'), 10);
		
		if (this.id == 'font-enlarge')
			FontSizeValue += 2;
		else if (this.id == 'font-shrink')
			FontSizeValue -= 2;
		else if (this.id == 'font-std')
			FontSizeValue = StdFontSize;
			
		if (FontSizeValue <= largestFontSize && FontSizeValue >= smallestFontSize){
			$('div.content_std h1').css('fontSize', (FontSizeValue + 10) + "px");
			$('h2').css('fontSize', (FontSizeValue + 6) + "px");
			$('h3').css('fontSize', (FontSizeValue + 10) + "px");
			$('a').css('fontSize',  (FontSizeValue + 1) + "px");
			$('p').css('fontSize', FontSizeValue + "px");
			//$.cookie('NewFontSize', FontSizeValue);
		}
	});
/*	
	if($.cookie('NewFontSize')){
		var FontSizeValue = parseInt($.cookie('NewFontSize'));
		$('div.content_std h1').css('fontSize', (FontSizeValue + 10) + "px");
		$('h2').css('fontSize', (FontSizeValue + 6) + "px");
		$('h3').css('fontSize', (FontSizeValue + 10) + "px");
		$('a').css('fontSize',  (FontSizeValue + 1) + "px");
		$('p').css('fontSize', FontSizeValue + "px");
	}
*/
});
