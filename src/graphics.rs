use alloc::vec::Vec;

use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::Drawable;
use embedded_graphics::geometry::{Dimensions, Point, Size};
use embedded_graphics::image::{Image, ImageDrawable};
use embedded_graphics::mono_font::{MonoFont, MonoTextStyle};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::Primitive;
use embedded_graphics::primitives::{PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, RoundedRectangle};
use embedded_graphics::text::{Alignment, Baseline, Text, TextStyle, TextStyleBuilder};
use embedded_graphics::text::renderer::TextRenderer;
use heapless::String;

pub struct GraphicUtils;

impl GraphicUtils {
    pub fn display_text<D, S>(display: &mut D, pos: Point, character_style: S,
                              text_style: TextStyle, text: &str) -> Result<Point, D::Error>
        where D: DrawTarget<Color=Rgb565>, S: TextRenderer<Color=Rgb565> {
        Text::with_text_style(
            text,
            pos,
            character_style,
            text_style,
        )
            .draw(display)
    }

    pub fn display_text_with_background<D, S>(display: &mut D, pos: Point, character_style: S,
                                              text_style: TextStyle, text: &str,
                                              background_style: PrimitiveStyle<Rgb565>,
                                              width: u32) -> Result<Point, D::Error>
        where D: DrawTarget<Color=Rgb565>, S: TextRenderer<Color=Rgb565> {
        let text = Text::with_text_style(
            text,
            pos,
            character_style,
            text_style,
        );
        Rectangle::new(pos, Size::new(width as u32, text.bounding_box().size.height))
            .into_styled(background_style)
            .draw(display)?;
        text.draw(display)
    }

    pub fn display_image_with_background<D, T>(display: &mut D, image: &Image<T>,
                                               background_style: PrimitiveStyle<Rgb565>) -> Result<(), D::Error>
        where D: DrawTarget<Color=Rgb565>, T: ImageDrawable<Color=Rgb565> {
        let bounding_box = image.bounding_box();
        Rectangle::new(bounding_box.top_left, bounding_box.size)
            .into_styled(background_style)
            .draw(display)?;

        image.draw(display)
    }

    pub fn get_button_size() -> Size {
        Size::new(90, 50)
    }
    pub fn get_text_with_ellipsis_from_string(width: u32, text: &str, font: &MonoFont) -> alloc::string::String {
        GraphicUtils::get_text_with_ellipsis_from_str(width, text, font)
    }
    pub fn get_text_with_ellipsis_from_str(width: u32, text: &str, font: &MonoFont) -> alloc::string::String {
        let text_width = font.character_size.width * text.len() as u32;
        if text_width > width {
            let text_len_visible = width / font.character_size.width;
            let text_visible = text.split_at((text_len_visible - 3) as usize).0;
            let mut text_visible_str = alloc::string::String::from(text_visible);
            text_visible_str.push_str("...");
            return text_visible_str;
        }
        alloc::string::String::from(text)
    }
}

pub trait ListItem {
    fn get_text(&self) -> &str;
    fn get_height(&self) -> u16;
    fn get_font(&self) -> &MonoFont<'_>;
    fn get_text_style(&self) -> TextStyle;
}

pub struct List<T> {
    list_items: Vec<T>,
    pos: Point,
    size: Size,
    selected_index: usize,
    visible_lines: usize,
    window_start: usize,
    highlight_color: Rgb565,
    background_color: Rgb565,
    text_color: Rgb565,
}

impl<T: ListItem + Clone> List<T> {
    pub fn new(items: &Vec<T>, pos: Point, size: Size, theme: &Theme) -> Self {
        List {
            list_items: items.clone(),
            pos,
            size,
            selected_index: 0,
            visible_lines: if items.len() == 0 { 1 } else { (size.height as u16 / items.first().unwrap().get_height()) as usize },
            window_start: 0,
            highlight_color: theme.highlight_color,
            background_color: theme.screen_background_color,
            text_color: theme.text_color_primary,
        }
    }

    fn get_selected_style(&self) -> PrimitiveStyle<Rgb565> {
        PrimitiveStyleBuilder::new()
            .fill_color(self.highlight_color)
            .build()
    }

    fn get_background_style(&self) -> PrimitiveStyle<Rgb565> {
        PrimitiveStyleBuilder::new()
            .fill_color(self.background_color)
            .build()
    }

    fn get_scrollbar_style(&self) -> PrimitiveStyle<Rgb565> {
        PrimitiveStyleBuilder::new()
            .fill_color(self.highlight_color)
            .build()
    }

    fn get_scrollbar_indicator_style(&self) -> PrimitiveStyle<Rgb565> {
        PrimitiveStyleBuilder::new()
            .fill_color(self.text_color)
            .build()
    }

    fn get_character_style<'a>(&self, item: &'a T) -> MonoTextStyle<'a, Rgb565> {
        MonoTextStyle::new(
            item.get_font(),
            self.text_color)
    }

    fn show_scrollbar(&self) -> bool {
        self.list_items.len() > self.visible_lines
    }

    fn get_scrollbar_width(&self) -> u32 {
        20u32
    }

    fn get_visible_text(&self, item: &T) -> alloc::string::String {
        let visible_width = self.size.width - self.get_scrollbar_width();
        GraphicUtils::get_text_with_ellipsis_from_string(visible_width, item.get_text(), item.get_font())
    }

    pub fn draw<D>(&self, display: &mut D) -> Result<(), D::Error>
        where D: DrawTarget<Color=Rgb565> {
        for list_items_index in self.window_start..(self.window_start + self.visible_lines).min(self.list_items.len()) {
            let text = self.get_visible_text(&self.list_items[list_items_index]);
            let item_height = self.list_items[list_items_index].get_height();
            let character_style = self.get_character_style(&self.list_items[list_items_index]);
            let text_style = self.list_items[list_items_index].get_text_style();
            let mut background_style = self.get_background_style();
            if self.selected_index == list_items_index {
                background_style = self.get_selected_style();
            }

            GraphicUtils::display_text_with_background(display, Point::new(self.pos.x, self.pos.y + ((list_items_index - self.window_start) * item_height as usize) as i32),
                                                       character_style, text_style, text.as_str(), background_style,
                                                       if self.show_scrollbar() { self.size.width - self.get_scrollbar_width() } else { self.size.width - 10 })?;
        }
        if self.show_scrollbar() {
            // scrollbar
            let scrollbar_height_absolute = self.size.height - 10;
            let scrollbar_pos = Point::new((self.size.width - self.get_scrollbar_width()) as i32, self.pos.y);
            let scrollbar_size = Size::new(self.get_scrollbar_width(), scrollbar_height_absolute);
            Rectangle::new(scrollbar_pos, scrollbar_size)
                .into_styled(self.get_scrollbar_style())
                .draw(display)?;

            let scrollbar_indicator_height = (scrollbar_height_absolute as f32 / (self.list_items.len() as f32 / self.visible_lines as f32)) as usize;
            let scrollbar_indicator_start = (scrollbar_height_absolute as f32 * (self.window_start as f32 / self.list_items.len() as f32)) as usize;
            let scrollbar_indicator_pos = Point::new((self.size.width - self.get_scrollbar_width()) as i32, self.pos.y + scrollbar_indicator_start as i32);
            let scrollbar_indicator_size = Size::new(self.get_scrollbar_width(), scrollbar_indicator_height as u32);
            Rectangle::new(scrollbar_indicator_pos, scrollbar_indicator_size)
                .into_styled(self.get_scrollbar_indicator_style())
                .draw(display)?;
        }
        Ok(())
    }

    pub fn scroll_down<D>(&mut self, display: &mut D) -> Result<(), D::Error>
        where D: DrawTarget<Color=Rgb565> {
        if self.selected_index < self.list_items.len() - 1 {
            self.selected_index += 1
        };
        if self.selected_index > self.window_start + self.visible_lines - 1 {
            self.window_start += 1;
        }

        self.draw(display)
    }

    pub fn scroll_up<D>(&mut self, display: &mut D) -> Result<(), D::Error>
        where D: DrawTarget<Color=Rgb565> {
        if self.selected_index > 0 {
            self.selected_index -= 1
        };
        if self.selected_index < self.window_start {
            self.window_start -= 1;
        }

        self.draw(display)
    }

    pub fn select_at_pos<D>(&mut self, display: &mut D, pos: Point) -> Result<usize, D::Error>
        where D: DrawTarget<Color=Rgb565> {
        for list_items_index in self.window_start..(self.window_start + self.visible_lines).min(self.list_items.len()) {
            let item_height = self.list_items[list_items_index].get_height();
            let item_pos = Point::new(self.pos.x, self.pos.y + ((list_items_index - self.window_start) * item_height as usize) as i32);
            let bounding_box = Rectangle::new(item_pos, Size::new(self.size.width, item_height as u32));
            if (bounding_box.contains(pos)) {
                self.selected_index = list_items_index;
                break;
            }
        }
        self.draw(display)?;
        Ok(self.selected_index)
    }

    pub fn get_selected_index(&self) -> usize {
        self.selected_index
    }

    pub fn set_selected_index(&mut self, index: usize) {
        if index >= 0 && index < self.list_items.len() {
            self.selected_index = index;
        }
    }

    pub fn get_bounding_box(&self) -> Rectangle {
        Rectangle::new(self.pos, self.size)
    }
}

pub struct Button<'a, T> {
    image: &'a T,
    pos: Point,
    size: Size,
}

impl<'a, T: ImageDrawable<Color=Rgb565>> Button<'a, T> {
    pub fn new(image_drawable: &'a T, position: Point) -> Self {
        Button {
            image: image_drawable,
            pos: position,
            size: GraphicUtils::get_button_size(),
        }
    }

    pub fn set_image_drawable(&mut self, image_drawable: &'a T) {
        self.image = image_drawable;
    }

    pub fn draw<D>(&self, display: &mut D, background_style: PrimitiveStyle<Rgb565>) -> Result<(), D::Error>
        where D: DrawTarget<Color=Rgb565> {
        let visible_pos = Point::new(self.pos.x + 5, self.pos.y + 5);
        let visible_size = Size::new(self.size.width - 10, self.size.height - 10);
        RoundedRectangle::with_equal_corners(Rectangle::new(visible_pos, visible_size), Size::new(10, 10))
            .into_styled(background_style)
            .draw(display)?;

        let image_margin_x = (visible_size.width - self.image.size().width) / 2;
        let image_margin_y = (visible_size.height - self.image.size().height) / 2;

        let image = Image::new(self.image, Point::new(visible_pos.x + image_margin_x as i32, visible_pos.y + image_margin_y as i32));
        image.draw(display)
    }
    pub fn get_bounding_box(&self) -> Rectangle {
        Rectangle::new(self.pos, self.size)
    }
}

pub struct Theme {
    pub button_background_color: Rgb565,
    pub button_foreground_color: Rgb565,
    pub screen_background_color: Rgb565,
    pub text_color_primary: Rgb565,
    pub highlight_color: Rgb565,
    pub error_color: Rgb565,
}

pub struct Progress<'a, T> {
    image_drawable: &'a T,
    text: String<256>,
    pos: Point,
    size: Size,
    background_color: Rgb565,
    foreground_color: Rgb565,
    character_style: MonoTextStyle<'a, Rgb565>,
}

impl<'a, T: ImageDrawable<Color=Rgb565>> Progress<'a, T> {
    pub fn new(image_drawable: &'a T, text: &str, position: Point, size: Size, background_color: Rgb565,
               character_style: MonoTextStyle<'a, Rgb565>, theme: &Theme) -> Self {
        Progress {
            image_drawable,
            text: String::from(text),
            pos: position,
            size,
            background_color,
            foreground_color: theme.text_color_primary,
            character_style,
        }
    }
    fn get_background_style(&self) -> PrimitiveStyle<Rgb565> {
        PrimitiveStyleBuilder::new()
            .fill_color(self.background_color)
            .build()
    }
    fn get_text_style(&self) -> TextStyle {
        TextStyleBuilder::new()
            .alignment(Alignment::Center)
            .baseline(Baseline::Top)
            .build()
    }

    pub fn update_text<D>(&mut self, display: &mut D, text: &str) -> Result<(), D::Error>
        where D: DrawTarget<Color=Rgb565> {
        self.text = String::from(text);
        self.draw(display)
    }

    pub fn draw<D>(&self, display: &mut D) -> Result<(), D::Error>
        where D: DrawTarget<Color=Rgb565> {
        Rectangle::new(self.pos, self.size)
            .into_styled(self.get_background_style())
            .draw(display)?;

        let text_height = self.character_style.font.character_size.height;

        let image_size = self.image_drawable.size();
        let image_pos_x = self.pos.x + ((self.size.width - image_size.width) / 2) as i32;
        let image_pos_y = self.pos.y + ((self.size.height - image_size.height) / 2 - text_height) as i32;
        let image = Image::new(self.image_drawable, Point::new(image_pos_x, image_pos_y));
        image.draw(display)?;

        let text_pos_x = self.pos.x + (self.size.width / 2) as i32;
        let text_pos_y = self.pos.y + image_pos_y + image_size.height as i32 + text_height as i32;
        let text = Text::with_text_style(
            self.text.as_str(),
            Point::new(text_pos_x, text_pos_y),
            self.character_style,
            self.get_text_style(),
        );
        let _ = text.draw(display);
        Ok(())
    }
}

pub struct Label<'a> {
    text: String<256>,
    pos: Point,
    width: u32,
    background_color: Rgb565,
    foreground_color: Rgb565,
    character_style: MonoTextStyle<'a, Rgb565>,
}

impl<'a> Label<'a> {
    pub fn new(text: &str, position: Point, width: u32, background_color: Rgb565,
               character_style: MonoTextStyle<'a, Rgb565>, theme: &Theme) -> Self {
        Label {
            text: String::from(text),
            pos: position,
            width,
            background_color,
            foreground_color: theme.text_color_primary,
            character_style,
        }
    }
    fn get_background_style(&self) -> PrimitiveStyle<Rgb565> {
        PrimitiveStyleBuilder::new()
            .fill_color(self.background_color)
            .build()
    }

    fn get_text_style(&self) -> TextStyle {
        TextStyleBuilder::new()
            .alignment(Alignment::Left)
            .baseline(Baseline::Top)
            .build()
    }
    pub fn draw<D>(&self, display: &mut D) -> Result<Point, D::Error>
        where D: DrawTarget<Color=Rgb565> {
        GraphicUtils::display_text_with_background(display, self.pos, self.character_style,
                                                   self.get_text_style(),
                                                   GraphicUtils::get_text_with_ellipsis_from_str(self.width, self.text.as_str(), self.character_style.font).as_str(),
                                                   self.get_background_style(), self.width)
    }

    pub fn update_text<D>(&mut self, display: &mut D, text: &str) -> Result<Point, D::Error>
        where D: DrawTarget<Color=Rgb565> {
        self.text = String::from(text);
        self.draw(display)
    }
}